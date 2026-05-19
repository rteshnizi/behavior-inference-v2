from dataclasses import dataclass, field


@dataclass
class TraversabilityRequirements:
	"""Constraints a spatial region places on traversing targets."""
	transportation_req: list[str] = field(default_factory=list)
	"""Allowed transportation modes. Empty list means unconstrained."""
	max_diameter: str = ""
	"""Maximum target diameter in categories."""
	max_height: str = ""
	"""Maximum clearance in categories."""
	iri: str = ""
	"""IRI of the TraversabilityReq individual in RDF. Empty means unconstrained."""

	@staticmethod
	def fromDict(d: dict) -> "TraversabilityRequirements":
		return TraversabilityRequirements(
			transportation_req=d.get("transportation_req", []),
			max_diameter=d.get("max_diameter", ""),
			max_height=d.get("max_height", ""),
			iri=d.get("iri", ""),
		)

	def asDict(self) -> dict:
		return {
			"transportation_req": self.transportation_req,
			"max_diameter": self.max_diameter,
			"max_height": self.max_height,
			"iri": self.iri,
		}
