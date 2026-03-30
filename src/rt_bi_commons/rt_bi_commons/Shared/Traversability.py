from dataclasses import dataclass, field
from typing import TypedDict

from rt_bi_commons.Utils import Ros


@dataclass
class TraversabilityRequirements:
	"""Constraints a spatial region places on traversing targets."""
	transportation_req: list[str] = field(default_factory=list)
	"""Allowed transportation modes. Empty list means unconstrained."""
	max_diameter: float = float("inf")
	"""Maximum target diameter in metres."""
	max_clearance: float = float("inf")
	"""Maximum clearance in metres."""

	@staticmethod
	def fromDict(d: dict) -> "TraversabilityRequirements":
		return TraversabilityRequirements(
			transportation_req=d.get("transportation_req", []),
			max_diameter=d.get("max_diameter", float("inf")),
			max_clearance=d.get("min_clearance", float("inf")),
		)

	def asDict(self) -> dict:
		return {
			"transportation_req": self.transportation_req,
			"max_diameter": self.max_diameter,
			"max_clearance": self.max_clearance,
		}


class TargetAttributes(TypedDict, total=False):
	"""
	Attributes known (or inferred) for a target hypothesis carried by a token.
	All keys are optional — an absent key means "don't care" (permissive).
	"""
	transportation_mode: list[str]
	"""Set of possible transportation modes for this hypothesis."""
	diameter: float
	"""Physical diameter of the target in metres."""
	height: float
	"""Physical height of the target in metres."""


def isCompatible(reqs: TraversabilityRequirements, attrs: TargetAttributes) -> bool:
	"""
	Return True when attrs are compatible with reqs.

	- If reqs is None the region is unconstrained → always compatible.
	- For each constraint present in reqs, if the token has the corresponding
	  attribute it must satisfy the constraint; absent attributes are ignored
	  (don't-care / permissive).
	"""
	if reqs is None:
		return True

	# Transportation mode check
	if reqs.transportation_req and "transportation_mode" in attrs:
		allowed = set(reqs.transportation_req)
		token_modes = set(attrs["transportation_mode"])
		if not token_modes.intersection(allowed):
			return False

	# Diameter check
	if reqs.max_diameter is not None and "diameter" in attrs:
		if attrs["diameter"] > reqs.max_diameter:
			return False

	# Clearance check
	if reqs.max_clearance is not None and "height" in attrs:
		if attrs["height"] > reqs.max_clearance:
			return False

	return True


def inferAttributes(reqs: TraversabilityRequirements, attrs: TargetAttributes) -> TargetAttributes:
	"""
	Return a copy of attrs enriched/narrowed by traversing a region with reqs.

	- If reqs is None, returns attrs unchanged.
	- If the token has no transportation_mode and the region constrains it,
	  the token inherits all allowed modes.
	- If the token already has transportation_mode, the list is narrowed to the
	  intersection with the region's allowed modes.
	- Numeric constraints (diameter, height) are not inferred — only an exact
	  known value could satisfy a max/min constraint.
	"""
	if reqs is None:
		return TargetAttributes(**attrs)  # type: ignore[misc]

	result = TargetAttributes(**attrs)  # type: ignore[misc]

	if reqs.transportation_req:
		allowed = set(reqs.transportation_req)
		if "transportation_mode" not in result:
			result["transportation_mode"] = list(allowed)
		else:
			narrowed = list(set(result["transportation_mode"]).intersection(allowed))
			result["transportation_mode"] = narrowed

	return result
