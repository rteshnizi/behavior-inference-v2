from dataclasses import dataclass, field
from typing import TypedDict


@dataclass
class TraversabilityRequirements:
	"""Constraints a spatial region places on traversing targets."""
	transportation_req: list[str] = field(default_factory=list)
	"""Allowed transportation modes. Empty list means unconstrained."""
	max_diameter: str = ""
	"""Maximum target diameter in categories."""
	max_height: str = ""
	"""Maximum clearance in categories."""

	@staticmethod
	def fromDict(d: dict) -> "TraversabilityRequirements":
		return TraversabilityRequirements(
			transportation_req=d.get("transportation_req", []),
			max_diameter=d.get("max_diameter", ""),
			max_height=d.get("max_height", ""),
		)

	def asDict(self) -> dict:
		return {
			"transportation_req": self.transportation_req,
			"max_diameter": self.max_diameter,
			"max_height": self.max_height,
		}


class TargetAttributes(TypedDict, total=False):
	"""
	Attributes known (or inferred) for a target hypothesis carried by a token.
	All keys are optional — an absent key means "don't care" (permissive).
	"""
	transportation_mode: list[str]
	"""Set of possible transportation modes for this hypothesis."""
	diameter_bound: str
	"""Physical diameter of the target in categories. e.g., narrow, medium, wide."""
	height_bound: str
	"""Physical height of the target in categories. e.g., low, medium, high."""


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
	# if reqs.max_diameter is not None and "diameter" in attrs:
	# 	if attrs["diameter"] > reqs.max_diameter:
	# 		return False

	# Clearance check
	# if reqs.max_height is not None and "height" in attrs:
	# 	if attrs["height"] > reqs.max_height:
	# 		return False

	return True
