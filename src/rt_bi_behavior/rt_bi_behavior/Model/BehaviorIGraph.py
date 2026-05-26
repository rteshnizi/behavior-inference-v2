from json import loads
from typing import Any, Callable, Optional, cast

import networkx as nx

from rt_bi_behavior.Model.Transition import TransitionStatement
from rt_bi_commons.Shared.NodeId import NodeId
from rt_bi_commons.Shared.Predicates import Predicates
from rt_bi_commons.Shared.Traversability import Attributes, AttributeValue
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_commons.Utils.NetworkX import NxUtils


class BehaviorIGraph(NxUtils.Graph):
	def __init__(self, g: nx.DiGraph | None = None):
		NxUtils.Graph.__init__(self, None)
		nx.DiGraph.__init__(self, g)

	def createNodeMarkers(self) -> list:
		return []

	def createEdgeMarkers(self) -> list:
		return []

	def satisfies(self, node: NodeId, criterion: TransitionStatement, token_id: str = "", checkSatisfiesFn: "Optional[Callable[[str, Any], bool]]" = None) -> bool:
		predicates = self.getContent(node, "predicates")
		return criterion.evaluate(predicates, token_id=token_id, checkSatisfiesFn=checkSatisfiesFn)

	def isTraversableBy(self, node: NodeId, attrs: Attributes) -> bool:
		"""Stub: traversability testing moves to SPARQL in step 7; currently returns True."""
		return True

	def deltaAttributes(self, node: NodeId) -> Attributes:
		"""Return the region's traversability restrictions as an Attributes instance.

		The BA asserts these on the new token's RDF row so the derived_from*
		chain accumulates the effective values at query time.
		"""
		d = self.nodes[node].get("traversability_reqs")
		if d is None: raise ValueError(f"Node {node} does not have traversability requirements")
		return Attributes.fromDict(d)

	def propagate(self, source: NodeId, visited: set[NodeId]) -> dict[NodeId, list[NodeId]]:
		"""Returns a dictionary from target Id to path"""
		if source not in self.nodes: return {}
		weightFn = lambda u, v, d: 1000000 if v in visited else 1
		# The return value is a tuple of two dictionaries keyed by target nodes. The first dictionary stores distance to each target node. The second stores the path to each target node.
		paths: tuple[dict[NodeId, float], dict[NodeId, list[NodeId]]] = nx.single_source_dijkstra(self, source, cutoff=1000001, weight=weightFn) # pyright: ignore[reportAssignmentType, reportArgumentType]
		destinations = paths[1]
		# Ros.Log("DIJKSTRA", paths, severity=Ros.LoggingSeverity.ERROR)
		# Ros.Log("DIJKSTRA", [(d, destinations[d]) for d in destinations], severity=Ros.LoggingSeverity.ERROR)
		return destinations

	def propagateOneStep(self, source: NodeId, visited: set[NodeId]) -> dict[NodeId, list[NodeId]]:
		if source not in self.nodes: return {}
		paths: dict[NodeId, list[NodeId]] = {}
		for destination in cast(list[NodeId], self[source]):
			if destination in visited: continue
			paths[destination] = [source, destination]
		return paths

	@classmethod
	def fromMsg(cls, msg: Msgs.RtBi.IGraph) -> "BehaviorIGraph":
		d: dict[str, Any] = loads(msg.adjacency_json)
		for node in d["nodes"]:
			node["id"] = NodeId.fromDict(node["id"])
			node["predicates"] = Predicates(node["predicates"])
			node["traversability_reqs"] = Attributes.fromDict(node["traversability_reqs"]).asDict()
			# traversability_reqs is stored as a plain dict produced by Attributes.asDict()
		for adj in d["adjacency"]:
			for edge in adj:
				edge["id"] = NodeId.fromDict(edge["id"])
		g = nx.adjacency_graph(d)
		return BehaviorIGraph(g)
