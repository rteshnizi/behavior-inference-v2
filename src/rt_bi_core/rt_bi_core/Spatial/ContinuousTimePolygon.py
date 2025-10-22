from math import isnan, nan
from typing import Any, Final, Generic, TypeVar, cast

from rt_bi_commons.Shared.Predicates import Predicates
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Geometry import AffineTransform, GeometryLib, Shapely
from rt_bi_core.Spatial import PolygonFactory, PolygonFactoryKeys
from rt_bi_core.Spatial.AffinePolygon import AffinePolygon
from rt_bi_core.Spatial.DynamicPolygon import DynamicPolygon
from rt_bi_core.Spatial.Polygon import Polygon
from rt_bi_core.Spatial.SensingPolygon import SensingPolygon
from rt_bi_core.Spatial.StaticPolygon import StaticPolygon
from rt_bi_core.Spatial.TargetPolygon import TargetPolygon
from rt_bi_core.Spatial.Tracklet import Tracklet

_T_Poly = TypeVar("_T_Poly", bound=Polygon)
class ContinuousTimePolygon(Generic[_T_Poly]):
	INF_NS: Final[int] = 2 ** 100 #9223372036854775808
	def __init__(self, polyConfigs: list[_T_Poly]) -> None:
		self.__sortedConfigs: list[_T_Poly] = []
		for poly in polyConfigs: self.addPolygon(poly, -1)
		return

	def __repr__(self) -> str:
		tStr = f"{self.earliestNanoSecs}, {self.latestNanoSecs}"
		if self.isProjective or self.earliestNanoSecs == self.latestNanoSecs:
			tStr = f"{self.earliestNanoSecs}"
		return f"CTR-{self.name}[{tStr}](#{self.length})"

	def __getitem__(self, timeNanoSecs: int) -> _T_Poly:
		"""Get polygonal shape at time.

		Parameters
		----------
		timeNanoSecs : int
			Time in NanoSeconds.

		Returns
		-------
		Shapely.Polygon
			The shape.
		"""
		index = self.timeNanoSecsToIndex(timeNanoSecs) # This tests for edge-cases as well
		if self.isProjective: return self.configs[0]
		if timeNanoSecs == self.earliestNanoSecs: return self.configs[0]
		if timeNanoSecs == self.latestNanoSecs: return self.configs[self.length - 1]

		param = self.__getParameterizedTime(index, timeNanoSecs)
		if isnan(param): return self.configs[index]

		transform = GeometryLib.getParameterizedAffineTransformation(self.__transformation(index), param)
		cor = GeometryLib.applyMatrixTransformToCoords(transform, self.configs[index].centerOfRotation)
		poly = GeometryLib.applyMatrixTransformToPolygon(transform, self.configs[index].interior)
		kwArgs: dict[PolygonFactoryKeys, Any] = {
			"polygonId": self.id.polygonId,
			"regionId": self.id.regionId,
			"subPartId": self.id.subPartId,
			"centerOfRotation": cor,
			"envelope": [],
			"timeNanoSecs": timeNanoSecs,
			"interior": poly,
			"hIndex": -1,
			"predicates": self.predicates(timeNanoSecs),
		}
		if self.type == SensingPolygon.type:
			kwArgs["tracklets"] = {}
			assert index < self.length - 1, "This index must be in the range at this point in the code based on the above checks"
			nextPoly = cast(SensingPolygon, self.configs[index + 1])
			prevPoly = cast(SensingPolygon, self.configs[index])
			for trackletId in nextPoly.tracklets:
				nextTracklet = nextPoly.tracklets[trackletId]
				if nextTracklet.entered: continue # because timeNanoSecs is before the enter event
				# assert trackletId in prevPoly.tracklets, "Track Id must be present in previous config based on the above checks"
				if trackletId not in prevPoly.tracklets: continue
				prevTracklet = prevPoly.tracklets[trackletId]
				kwArgs["tracklets"][trackletId] = self.__interpolateTrack(prevTracklet, nextTracklet, timeNanoSecs)
			Cls = SensingPolygon
		elif self.type == StaticPolygon.type:
			kwArgs["timeNanoSecs"] = self.configs[index].timeNanoSecs
			Cls = StaticPolygon
		elif self.type == DynamicPolygon.type:
			Cls = DynamicPolygon
		elif self.type == AffinePolygon.type:
			Cls = AffinePolygon
		elif self.type == TargetPolygon.type:
			Cls = AffinePolygon
		else:
			raise AssertionError(f"Unexpected region type: {repr(self.type)} in {self.name}")
		poly = PolygonFactory(Cls, kwArgs)
		return cast(_T_Poly, poly)

	def __contains__(self, timeNanoSecs: int) -> bool:
		if self.earliestNanoSecs == -1: return False
		if self.latestNanoSecs == -1: return False
		if timeNanoSecs < self.earliestNanoSecs: return False
		if timeNanoSecs > self.latestNanoSecs: return False
		return True

	def predicates(self, timeNanoSecs: int) -> Predicates:
		if self.length == 0: return Predicates([])
		i = self.timeNanoSecsToIndex(timeNanoSecs)
		return self.configs[i].predicates

	@property
	def name(self) -> str:
		if self.length == 0: return self.type.value
		(regionId, polyId) = self.configs[0].id.shortNames()
		name = f"{self.type.value}-{regionId}-{polyId}"
		return name

	@property
	def id(self) -> AffinePolygon.Id:
		if self.length == 0: return AffinePolygon.Id(hIndex=-1, timeNanoSecs=-1, regionId="", polygonId="", subPartId="")
		return self.configs[0].id

	@property
	def length(self) -> int:
		return len(self.__sortedConfigs)

	@property
	def configs(self) -> list[_T_Poly]:
		return self.__sortedConfigs

	@property
	def type(self) -> AffinePolygon.Types:
		if self.length == 0: return AffinePolygon.Types.BASE
		return self.configs[0].type

	@property
	def latestNanoSecs(self) -> int:
		if self.length == 0: return -1
		if self.isProjective: return self.INF_NS
		return self.__sortedConfigs[-1].timeNanoSecs

	@property
	def earliestNanoSecs(self) -> int:
		if self.length == 0: return -1
		return self.__sortedConfigs[0].timeNanoSecs

	@property
	def isSlice(self) -> bool:
		return self.earliestNanoSecs == self.latestNanoSecs and self.earliestNanoSecs != -1

	@property
	def isProjective(self) -> bool:
		return self.type == StaticPolygon.type or self.type == DynamicPolygon.type

	def timeNanoSecsToIndex(self, timeNanoSecs: int) -> int:
		if self.length == 0: raise ValueError("Empty CTR.")
		if self.earliestNanoSecs == -1: raise ValueError(f"earliestNanoSecs is not set in {self.name}.")
		if self.latestNanoSecs == -1: raise ValueError(f"latestNanoSecs is not set in {self.name}.")

		if self.isProjective: return 0

		if timeNanoSecs < self.earliestNanoSecs: raise IndexError(f"{timeNanoSecs} is less than earliestNanoSecs={self.earliestNanoSecs} in {self.name}.")
		if timeNanoSecs > self.latestNanoSecs: raise IndexError(f"{timeNanoSecs} is greater than latestNanoSecs={self.latestNanoSecs} in {self.name}.")
		i = 0
		for i in range(self.length):
			if timeNanoSecs < self.configs[i].timeNanoSecs: break
		return i - 1

	@classmethod
	def __interpolateTrack(cls, prevTracklet: Tracklet, nextTracklet: Tracklet, eventTimeNs: int) -> Tracklet:
		if prevTracklet.id != nextTracklet.id: raise ValueError("Tracklets don't have the same id.. prevId = %d, currentId = %d" % (prevTracklet.id, nextTracklet.id))
		if eventTimeNs == prevTracklet.timeNanoSecs: return prevTracklet
		if eventTimeNs == nextTracklet.timeNanoSecs: return nextTracklet
		eventFraction = int((eventTimeNs - prevTracklet.timeNanoSecs) / (nextTracklet.timeNanoSecs - prevTracklet.timeNanoSecs))
		x = ((eventFraction * (nextTracklet.x - prevTracklet.x)) + prevTracklet.x)
		y = ((eventFraction * (nextTracklet.y - prevTracklet.y)) + prevTracklet.y)
		psi = ((eventFraction * (nextTracklet.psi - prevTracklet.psi)) + prevTracklet.psi)
		return Tracklet(nextTracklet.id, eventTimeNs, nextTracklet.hIndex, x, y, psi)

	def __getParameterizedTime(self, index: int, timeNanoSecs: int) -> float:
		"""### Get Parameterized Time
		Given a time, returns it as a fraction of the interval between `self.__configs[i]` and `self.__configs[i + 1]`.

		Parameters
		----------
		i : int
			The index of the lower bound of time in `self.__configs`
		timeNanoSecs : int
			ROS time in Nanoseconds

		Returns
		-------
		float
			A number in the range `[0, 1]`, or `nan` if the two ends of the interval are the same.
		"""
		if self.length < 2: return nan
		frac = timeNanoSecs - self.configs[index].timeNanoSecs
		total = self.configs[index + 1].timeNanoSecs - self.configs[index].timeNanoSecs
		if total == 0: return nan
		return (float(frac) / float(total))

	def __transformation(self, index: int) -> AffineTransform:
		return GeometryLib.getAffineTransformation(self.configs[index].envelope, self.configs[index + 1].envelope)

	def __transformationAt(self, timeNanoSecs: int) -> AffineTransform:
		assert timeNanoSecs in self, f"Requested time {timeNanoSecs} is out of range: {self.name} -- {self.earliestNanoSecs}-{self.latestNanoSecs}"
		if self.length == 1: return GeometryLib.getAffineTransformation(None, None)

		index = self.timeNanoSecsToIndex(timeNanoSecs)
		param = self.__getParameterizedTime(index, timeNanoSecs)
		if isnan(param): return GeometryLib.getAffineTransformation(None, None)
		transform = self.__transformation(index)
		transformationAtT = GeometryLib.getParameterizedAffineTransformation(transform, param)
		return transformationAtT

	def getEdgeBb(self, edge: Shapely.LineString, upToNs: int) -> Shapely.Polygon | Shapely.LineString:
		"""Get Edge Bounding Box

		:param str eName: edge name
		:return: The tightest bounding box around the given line segment.
		:rtype: `Shapely.Polygon` or `Shapely.LineString`
		"""
		if self.isProjective: return edge
		index = self.timeNanoSecsToIndex(upToNs)
		transform = self.__transformationAt(upToNs)
		return GeometryLib.getLineSegmentExpandedBb(transform, edge, self.configs[index].centerOfRotation)

	def getEdgeAt(self, edge: Shapely.LineString, timeNanoSecs: int) -> Shapely.LineString:
		if self.isProjective or self.length == 1: return edge
		transform = self.__transformationAt(timeNanoSecs)
		return GeometryLib.applyMatrixTransformToLineString(transform, edge)

	def addPolygon(self, polygon: _T_Poly, keepFromNs: int = -1) -> None:
		"""This method grabs any missing predicate from the previous layer.

		:param polygon: The polygon to be added.
		:param int keepFromNs: Any configuration prior to this time will be removed from the history,
		EXCEPT the last update before this stamp, defaults to -1 (keeps everything).
		"""
		if self.length > 0:
			assert self.id.copy(timeNanoSecs=-1, hIndex=-1) == polygon.id.copy(timeNanoSecs=-1, hIndex=-1), (
				f"Different ids in poly configs. {self.id} vs {polygon.id}. " +
				"A ContinuousTimeRegion must describe the evolution of a single polygon."
			)
			polygon.predicates = self.__sortedConfigs[-1].predicates.update(polygon.predicates)
		self.__sortedConfigs.append(polygon)
		self.__sortedConfigs = sorted(self.__sortedConfigs, key=lambda p: p.timeNanoSecs)

		while self.length > 2 and self.__sortedConfigs[0].timeNanoSecs < keepFromNs:
			if self.length == 1: return
			if self.__sortedConfigs[1].timeNanoSecs > keepFromNs: return
			c = self.__sortedConfigs.pop(0)
			Ros.Log(f"Dropped {c.timeNanoSecs} from {repr(self)}: Cutoff = {keepFromNs}.")
		return
