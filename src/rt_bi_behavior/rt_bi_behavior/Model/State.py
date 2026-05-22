from typing import TypedDict

from rt_bi_commons.Shared.NodeId import NodeId
from math import nan
from rt_bi_commons.Shared.Traversability import Attributes, AttributeValue, DiscreteAttribute, RangedAttribute
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Msgs import Msgs


class Token(TypedDict):
	id: str
	path: list[NodeId]

def tokenToMsg(token: Token, parentId: str = "", attributes: Attributes | None = None) -> Msgs.RtBi.Token:
	tokenMsg = Msgs.RtBi.Token(id=token["id"], parent_id=parentId)
	if attributes:
		for name, val in attributes.items.items():
			if isinstance(val, DiscreteAttribute):
				attr = Msgs.RtBi.Attribute(name=name, kind="discrete")
				attr.discrete_values = list(val.discrete_value)
				Ros.AppendMessage(tokenMsg.attributes, attr)
			elif isinstance(val, RangedAttribute):
				attr = Msgs.RtBi.Attribute(
					name=name, kind="ranged",
					range_min=val.range_min if val.range_min is not None else nan,
					range_max=val.range_max if val.range_max is not None else nan,
				)
				Ros.AppendMessage(tokenMsg.attributes, attr)
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
