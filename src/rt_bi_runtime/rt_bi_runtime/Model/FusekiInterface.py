from math import nan
from typing import Literal, TypeAlias, TypedDict, overload

import requests

from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Msgs import Msgs


class SparqlData(TypedDict):
	type: str
	value: str

class SparqlResultHelper:
	def __init__(self, response: requests.Response) -> None:
		self.variables: list[str] = []
		self.results: list[dict[str, SparqlData]] = []
		response.raise_for_status()
		# From this point on we can assume the request succeeded.
		j = response.json()
		self.variables = j["head"]["vars"]
		self.results = j["results"]["bindings"]

	def __getitem__(self, index: int) -> dict[str, SparqlData]:
		return self.results[index]

	def __len__(self) -> int:
		return len(self.results)

	def __repr__(self) -> str:
		return repr(self.results)

	def __isVariable(self, varName: str) -> bool:
		return varName in self.variables

	@overload
	def contains(self, i: int, var: list[str]) -> bool: ...

	@overload
	def contains(self, i: int, var: str) -> bool: ...

	def contains(self, i: int, var: str | list[str]) -> bool:
		if isinstance(var, list):
			return all(n in self[i] for n in var)
		return var in self[i]

	def boolVarValue(self, i: int, varName: str) -> bool:
		strVal = self.strVarValue(i, varName)
		if strVal == Msgs.RtBi.Predicate.TRUE: return True
		if strVal == Msgs.RtBi.Predicate.FALSE: return False
		if strVal == "": return False
		raise ValueError(f"Expected \"true\", \"false\", or \"\". Value is \"{strVal}\"")

	def floatVarValue(self, i: int, varName: str) -> float:
		strVal = self.strVarValue(i, varName)
		if strVal is not None: return float(strVal)
		return nan

	def strVarValue(self, i: int, varName: str) -> str:
		if not self.__isVariable(varName):
			raise KeyError(f"No variable with name\"{varName}\" defined.")
		if varName in self[i]:
			return self[i][varName]["value"]
		return ""

class FusekiInterface:
	SetTypes: TypeAlias = Literal["static", "dynamic", "affine", "temporal"]
	def __init__(self, node: Ros.Node, fusekiServerAdr: str, rdfStoreName: str) -> None:
		self.__fusekiServerAdr = fusekiServerAdr
		self.__rdfStoreName = rdfStoreName
		self.__node = node
		return

	@property
	def __SPARQL_URL(self) -> str:
		"""
		Creates URL from ROS parameters `fuseki_server` and `rdf_store`.
		Example: `http://192.168.50.164:8090/rt-bi/`
		"""
		return f"{self.__fusekiServerAdr}/{self.__rdfStoreName}/"

	def __parseIntervals(self, helper: SparqlResultHelper, i: int, regularSetId: str) -> tuple[list[Msgs.RtBi.TimeInterval], int]:
		intervals = []
		while i < len(helper):
			if helper.strVarValue(i, "regularSetId") != regularSetId:
					return (intervals, i)
			intervals.append(Msgs.RtBi.TimeInterval(
				min=Msgs.toTimeMsg(helper.floatVarValue(i, "min")),
				max=Msgs.toTimeMsg(helper.floatVarValue(i, "max")),
				include_min=helper.boolVarValue(i, "include_min"),
				include_max=helper.boolVarValue(i, "include_max"),
			))
			i += 1
		return (intervals, i)

	def __parseGeometry(self, helper: SparqlResultHelper, i: int, regularSetId: str) -> tuple[list[Msgs.RtBi.Polygon], int]:
		polys = []
		polyMsg = Msgs.RtBi.Polygon()
		polyMsg.id = helper.strVarValue(i, "polygonId")
		while i < len(helper):
			if helper.strVarValue(i, "polygonId") != polyMsg.id:
				polys.append(polyMsg)
				if helper.strVarValue(i, "regularSetId") != regularSetId:
					return (polys, i)
				polyMsg = Msgs.RtBi.Polygon()
				polyMsg.id = helper.strVarValue(i, "polygonId")
			x = helper.floatVarValue(i, "x")
			y = helper.floatVarValue(i, "y")
			Ros.AppendMessage(polyMsg.region.points, Msgs.Geometry.Point32(x=x, y=y, z=0.0))
			i += 1
		polys.append(polyMsg)
		return (polys, i)

	def __parsePredicates(self, helper: SparqlResultHelper, i: int) -> list[Msgs.RtBi.Predicate]:
		predicates = []
		for var in helper.variables:
			if not var.startswith("p_"): continue
			predicate = Msgs.RtBi.Predicate()
			predicate.name = var
			if helper.contains(i, var):
				val = helper.strVarValue(i, var)
				if val != Msgs.RtBi.Predicate.TRUE and val != Msgs.RtBi.Predicate.FALSE:
					raise ValueError(f"Unexpected predicate value: {val}")
				if val == Msgs.RtBi.Predicate.TRUE:
					predicate.value = val
					predicates.append(predicate)
		return predicates

	def __inferSetType(self, helper: SparqlResultHelper, i: int, msg: Msgs.RtBi.RegularSet) -> tuple["FusekiInterface.SetTypes", Msgs.RtBi.RegularSet]:
		addTo: list[FusekiInterface.SetTypes] = []
		if helper.boolVarValue(i, "static"):
			addTo.append("static")
			msg.set_type = Msgs.RtBi.RegularSet.STATIC
		if helper.boolVarValue(i, "dynamic"):
			addTo.append("dynamic")
			msg.set_type = Msgs.RtBi.RegularSet.DYNAMIC
		if helper.boolVarValue(i, "affine"):
			addTo.append("affine")
			msg.set_type = Msgs.RtBi.RegularSet.AFFINE
		if helper.boolVarValue(i, "temporal"):
			addTo.append("temporal")
			msg.set_type = Msgs.RtBi.RegularSet.TEMPORAL
		if len(addTo) != 1:
			raise RuntimeError(f"Regular set index {i} is {addTo}. A set must belong to exactly one group. Data: {repr(helper[i])}")
		return (addTo[0], msg)

	def __sendQuery(self, query: str) -> SparqlResultHelper:
		Ros.Log(f"Sending Query to Fuseki:\n{query}")
		response = requests.post(self.__SPARQL_URL, data={ "query": query })
		try:
			parsedResponse = SparqlResultHelper(response)
			Ros.Log(f"SPARQL Response:\n{parsedResponse}")
			return parsedResponse
		except Exception as e:
			Ros.Logger().error(f"SPARQL request failed with the following message: {repr(e)}")
			raise e

	__VALUE_IRI_PREFIXES: dict[str, str] = {
		"transportation_mode": "https://rezateshnizi.com/env/transportations#",
	}
	__TARGET_PREFIX = "https://rezateshnizi.com/env/targets#"
	__PROPERTY_PREFIX = "https://rezateshnizi.com/rt-bi-v2/property#"
	__CLASS_PREFIX = "https://rezateshnizi.com/rt-bi-v2/class#"

	def __sendUpdate(self, update: str) -> None:
		Ros.Log(f"Sending Update to Fuseki", [update])
		response = requests.post(self.__SPARQL_URL, data={ "update": update })
		response.raise_for_status()

	def insertToken(self, tokenId: str, parentId: str, attributes: list) -> None:
		"""Inserts a token as a class:Target individual into the knowledge base via INSERT DATA.

		attributes is a list of Attribute.msg objects (kind=="discrete" or "ranged").
		Each attribute is serialized as a reified blank node matching the ontology shape.
		"""
		import math as _math
		tokenIri = f"<{self.__TARGET_PREFIX}{tokenId}>"
		bnodeBase = tokenId.replace("#", "_").replace(":", "_").replace("/", "_")
		triples = [f"\t{tokenIri} a <{self.__CLASS_PREFIX}Target> ."]
		if parentId:
			parentIri = f"<{self.__TARGET_PREFIX}{parentId}>"
			triples.append(f"\t{tokenIri} <{self.__PROPERTY_PREFIX}derived_from> {parentIri} .")
		for attr in attributes:
			propIri = f"<{self.__PROPERTY_PREFIX}{attr.name}>"
			bnode = f"_:b_{attr.name}_{bnodeBase}"
			if attr.kind == "discrete":
				triples.append(f"\t{tokenIri} {propIri} {bnode} .")
				triples.append(f"\t{bnode} a <{self.__CLASS_PREFIX}DiscreteAttribute> .")
				for dv in attr.discrete_values:
					if not dv: continue
					if attr.name in self.__VALUE_IRI_PREFIXES:
						valueIri = f"<{self.__VALUE_IRI_PREFIXES[attr.name]}{dv}>"
					else:
						valueIri = f"<{dv}>"
					triples.append(f"\t{bnode} <{self.__PROPERTY_PREFIX}discrete_value> {valueIri} .")
			elif attr.kind == "ranged":
				triples.append(f"\t{tokenIri} {propIri} {bnode} .")
				triples.append(f"\t{bnode} a <{self.__CLASS_PREFIX}RangedAttribute> .")
				XSD = "http://www.w3.org/2001/XMLSchema#decimal"
				if not _math.isnan(attr.range_min):
					triples.append(f'\t{bnode} <{self.__PROPERTY_PREFIX}range_min> "{attr.range_min}"^^<{XSD}> .')
				if not _math.isnan(attr.range_max):
					triples.append(f'\t{bnode} <{self.__PROPERTY_PREFIX}range_max> "{attr.range_max}"^^<{XSD}> .')
		triplesStr = "\n".join(triples)
		sparql = f"INSERT DATA {{\n{triplesStr}\n}}"
		self.__sendUpdate(sparql)

	def fetchGeometryById(self, query: str, msgsToUpdate: dict[str, Msgs.RtBi.RegularSet]) -> dict[str, Msgs.RtBi.RegularSet]:
		resultHelper = self.__sendQuery(query)
		i = 0
		while i < len(resultHelper):
			setId = resultHelper.strVarValue(i, "regularSetId")
			msg = msgsToUpdate[setId]
			(polyMsgs, i) = self.__parseGeometry(resultHelper, i, setId)
			Ros.ConcatMessageArray(msg.polygons, polyMsgs)
		return msgsToUpdate

	def fetchIntervalsById(self, query: str, msgsToUpdate: dict[str, Msgs.RtBi.RegularSet]) -> dict[str, Msgs.RtBi.RegularSet]:
		resultHelper = self.__sendQuery(query)
		i = 0
		while i < len(resultHelper):
			setId = resultHelper.strVarValue(i, "regularSetId")
			msg = msgsToUpdate[setId]
			(intervals, i) = self.__parseIntervals(resultHelper, i, msg.id)
			Ros.ConcatMessageArray(msg.intervals, intervals)
		return msgsToUpdate

	def fetchTargets(self, query: str) -> dict[str, dict[str, str]]:
		"""Returns a mapping from target IRI to its effective attribute values.
		Outer key = the target IRI bound to ?target. Inner dict keys are the other
		variable names projected by the assembled query (e.g., \"diameter_bound\",
		\"transportation_mode\"); values are the SPARQL bindings, with empty strings
		for absent bindings (signaling \"unrestricted\" downstream)."""
		resultHelper = self.__sendQuery(query)
		result: dict[str, dict[str, str]] = {}
		for i in range(len(resultHelper)):
			targetIri = resultHelper.strVarValue(i, "target")
			result[targetIri] = {
				var: resultHelper.strVarValue(i, var)
				for var in resultHelper.variables
				if var != "target"
			}
		return result

	def fetchAggregationOperators(self, query: str) -> dict[str, str]:
		"""Returns a mapping from property IRI to operator IRI local-part (e.g., \"intersection\").
		The local-part doubles as the operator template filename basename."""
		resultHelper = self.__sendQuery(query)
		mapping: dict[str, str] = {}
		for i in range(len(resultHelper)):
			propIri = resultHelper.strVarValue(i, "prop")
			opIri = resultHelper.strVarValue(i, "op")
			mapping[propIri] = opIri.split("#")[-1]
		return mapping

	def fetchAttributeKinds(self, query: str) -> dict[str, "Literal['discrete', 'ranged']"]:
		"""Returns a mapping from outer property IRI to attribute kind (\"discrete\" or \"ranged\").
		Populated from attribute_kinds.rq at cold-start; used by __bootstrapAttributeKindCache."""
		resultHelper = self.__sendQuery(query)
		mapping: "dict[str, Literal['discrete', 'ranged']]" = {}
		for i in range(len(resultHelper)):
			propIri = resultHelper.strVarValue(i, "prop")
			kindIri = resultHelper.strVarValue(i, "kind")
			localPart = kindIri.split("#")[-1]
			if localPart == "DiscreteAttribute":
				mapping[propIri] = "discrete"
			elif localPart == "RangedAttribute":
				mapping[propIri] = "ranged"
		return mapping

	def askSatisfies(self, query: str) -> bool:
		"""Executes an ASK query (satisfies.rq filled by RdfStoreNode) and returns the boolean result."""
		Ros.Log(f"Sending ASK Satisfies query to Fuseki:\n{query}")
		response = requests.post(self.__SPARQL_URL, data={"query": query})
		response.raise_for_status()
		j = response.json()
		Ros.Log(f"ASK Satisfies response:\n{j}")
		return bool(j.get("boolean", False))

	def fetchSets(self, query: str) -> dict["FusekiInterface.SetTypes", dict[str, Msgs.RtBi.RegularSet]]:
		resultHelper = self.__sendQuery(query)
		stamp = Ros.Now(self.__node).to_msg()
		setsByType: dict[FusekiInterface.SetTypes, dict[str, Msgs.RtBi.RegularSet]] = {
			"static": {},
			"dynamic": {},
			"affine": {},
			"temporal": {},
		}
		# Group rows by regularSetId to handle multi-valued allowedTransport
		i = 0
		while i < len(resultHelper):
			regularSetId = resultHelper.strVarValue(i, "regularSetId")
			msg = Msgs.RtBi.RegularSet()
			msg.id = regularSetId
			msg.stamp = stamp
			(setType, msg) = self.__inferSetType(resultHelper, i, msg)
			msg.predicates = self.__parsePredicates(resultHelper, i)
			# Collect traversability restrictions — allowedTransport may span multiple rows;
			# range fields are single-valued (first non-empty binding wins).
			transport_values: list[str] = []
			diameter_min = diameter_max = height_min = height_max = ""
			while i < len(resultHelper) and resultHelper.strVarValue(i, "regularSetId") == regularSetId:
				transport = resultHelper.strVarValue(i, "allowedTransport")
				if transport:
					transport_values.append(transport.split("#")[-1])
				if not diameter_min: diameter_min = resultHelper.strVarValue(i, "minDiameter")
				if not diameter_max: diameter_max = resultHelper.strVarValue(i, "maxDiameter")
				if not height_min: height_min = resultHelper.strVarValue(i, "minHeight")
				if not height_max: height_max = resultHelper.strVarValue(i, "maxHeight")
				i += 1
			import math as _math
			restrictions: list[Msgs.RtBi.Attribute] = []
			if transport_values:
				_a = Msgs.RtBi.Attribute(name="transportation_mode", kind="discrete")
				_a.discrete_values = transport_values
				restrictions.append(_a)
			if diameter_min or diameter_max:
				_a = Msgs.RtBi.Attribute(
					name="diameter", kind="ranged",
					range_min=float(diameter_min) if diameter_min else _math.nan,
					range_max=float(diameter_max) if diameter_max else _math.nan,
				)
				restrictions.append(_a)
			if height_min or height_max:
				_a = Msgs.RtBi.Attribute(
					name="height", kind="ranged",
					range_min=float(height_min) if height_min else _math.nan,
					range_max=float(height_max) if height_max else _math.nan,
				)
				restrictions.append(_a)
			Ros.ConcatMessageArray(msg.traversability_restrictions, restrictions)
			if setType == "static" or setType == "dynamic":
				# Static Map is always reachable
				msg.predicates.append(Msgs.RtBi.Predicate(name="accessible", value=Msgs.RtBi.Predicate.TRUE))
			setsByType[setType][msg.id] = msg
		return setsByType
