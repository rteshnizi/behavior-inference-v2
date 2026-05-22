from dataclasses import dataclass, field
from math import isnan
from typing import TypeAlias


@dataclass
class DiscreteAttribute:
	kind: str = "discrete"
	discrete_value: list[str] = field(default_factory=list)


@dataclass
class RangedAttribute:
	kind: str = "ranged"
	range_min: float | None = None
	range_max: float | None = None


AttributeValue: TypeAlias = DiscreteAttribute | RangedAttribute


@dataclass
class Attributes:
	"""Map of outer attribute name → its reified value.

	Used identically by a target hypothesis (TargetAttributes) and by a
	region's restrictions (TraversabilityRestrictions); the two aliases below
	distinguish call sites for clarity.
	"""
	items: dict[str, AttributeValue] = field(default_factory=dict)

	@staticmethod
	def fromDict(d: dict) -> "Attributes":
		"""Deserialize from the JSON-friendly dict produced by :meth:`asDict`."""
		items: dict[str, AttributeValue] = {}
		for name, val in d.get("items", {}).items():
			kind = val.get("kind", "")
			if kind == "discrete":
				items[name] = DiscreteAttribute(discrete_value=list(val.get("discrete_value", [])))
			elif kind == "ranged":
				items[name] = RangedAttribute(
					range_min=val.get("range_min"),
					range_max=val.get("range_max"),
				)
		return Attributes(items=items)

	def asDict(self) -> dict:
		"""Serialize to a JSON-friendly dict (used when storing in iGraph node data)."""
		result: dict = {}
		for name, val in self.items.items():
			if isinstance(val, DiscreteAttribute):
				result[name] = {"kind": "discrete", "discrete_value": list(val.discrete_value)}
			elif isinstance(val, RangedAttribute):
				result[name] = {"kind": "ranged", "range_min": val.range_min, "range_max": val.range_max}
		return {"items": result}

	@staticmethod
	def fromMsgArray(attrs: list) -> "Attributes":
		"""Build :class:`Attributes` from a list of ``Attribute.msg`` objects."""
		items: dict[str, AttributeValue] = {}
		for attr in attrs:
			if attr.kind == "discrete":
				items[attr.name] = DiscreteAttribute(discrete_value=list(attr.discrete_values))
			elif attr.kind == "ranged":
				items[attr.name] = RangedAttribute(
					range_min=None if isnan(attr.range_min) else attr.range_min,
					range_max=None if isnan(attr.range_max) else attr.range_max,
				)
		return Attributes(items=items)


TargetAttributes: TypeAlias = Attributes
TraversabilityRestrictions: TypeAlias = Attributes
