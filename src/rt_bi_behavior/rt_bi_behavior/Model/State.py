from typing import TypedDict

from rt_bi_commons.Shared.NodeId import NodeId
from rt_bi_commons.Shared.Traversability import TargetAttributes
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Msgs import Msgs


class Token(TypedDict):
	id: str
	path: list[NodeId]

def tokenToMsg(token: Token, parentId: str = "", attributes: TargetAttributes | None = None) -> Msgs.RtBi.Token:
	tokenMsg = Msgs.RtBi.Token(id=token["id"], parent_id=parentId)
	if attributes:
		for key, val in attributes.items():
			if key == "transportation_mode":
				for mode in val: # pyright: ignore[reportGeneralTypeIssues]
					pred = Msgs.RtBi.Predicate(name="transportation_mode", value=mode)
					Ros.AppendMessage(tokenMsg.attributes, pred)
			elif isinstance(val, float):
				pred = Msgs.RtBi.Predicate(name=key, value=str(val))
				Ros.AppendMessage(tokenMsg.attributes, pred)
			elif isinstance(val, str):
				pred = Msgs.RtBi.Predicate(name=key, value=val)
				Ros.AppendMessage(tokenMsg.attributes, pred)
	return tokenMsg


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
