from typing import TypedDict

from rt_bi_commons.Shared.NodeId import NodeId
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Msgs import Msgs


class Token(TypedDict):
	id: str
	path: list[NodeId]

def tokenToMsg(token: Token, parentId: str = "", categories: list[str] | None = None) -> Msgs.RtBi.Token:
	tokenMsg = Msgs.RtBi.Token(id=token["id"], parent_id=parentId)
	if categories:
		for cat_iri in categories:
			pred = Msgs.RtBi.Predicate(name="target_category", value=cat_iri)
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
