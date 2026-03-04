from typing import TypedDict

from rt_bi_commons.Shared.NodeId import NodeId
from rt_bi_commons.Shared.Traversability import TargetAttributes


class Token(TypedDict):
	id: str
	path: list[NodeId]
	attributes: TargetAttributes

class TokenWithoutHistory(TypedDict):
	id: str
	iGraphNode: NodeId

class StateWithoutHistory(TypedDict):
	label: str
	tokens: list[TokenWithoutHistory]
	style: str
	fillcolor: str

class State(TypedDict):
	label: str
	tokens: list[Token]
	style: str
	fillcolor: str
