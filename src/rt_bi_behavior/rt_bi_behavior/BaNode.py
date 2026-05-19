from json import loads, dumps

from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.parameter import Parameter
import threading

from rt_bi_behavior import package_name
from rt_bi_behavior.Model.BehaviorIGraph import BehaviorIGraph
from rt_bi_behavior.Model.PropositionalBA import PropositionalBA
from rt_bi_commons.Base.ColdStartableNode import ColdStartable, CustomPayload
from rt_bi_commons.Base.RtBiNode import RtBiNode
from rt_bi_commons.Shared.NodeId import NodeId
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces


class BaNode(ColdStartable):
	"""
	This Node listens to all the messages published on the topics related to the Behavior Automaton.
	This node combines topic listeners and service clients.
	"""
	def __init__(self, **kwArgs) -> None:
		""" Create a Behavior Automaton node. """
		newKw = { "node_name": "ba", "loggingSeverity": Ros.LoggingSeverity.WARN, **kwArgs}
		RtBiNode.__init__(self, **newKw)
		ColdStartable.__init__(self)
		self.declareParameters()
		self.__name: str = self.get_fully_qualified_name()
		self.__baseDir = get_package_share_directory(package_name)
		self.__grammarDir: str = ""
		self.__grammarFile: str = ""
		self.__states: list[str] = []
		self.__transitions: dict[str, dict[str, str]] = {}
		self.__start: str = ""
		self.__accepting: list[str] = []
		self.__tokensCbGroup = ReentrantCallbackGroup()
		self.__tokensClient = self.create_client(
			Msgs.RtBiSrv.Tokens,
			RtBiInterfaces.ServiceNames.RT_BI_RUNTIME_DD_RDF_TARGETS.value,
			callback_group=self.__tokensCbGroup,
		)
		Ros.WaitForServiceToStart(self, self.__tokensClient)
		self.parseParameters()
		self.__tokenPublisher = RtBiInterfaces.createTokenPublisher(self)
		self.__ba = PropositionalBA(
			self.__name,
			self.__states,
			self.__transitions,
			self.__start,
			self.__accepting,
			self.__baseDir,
			self.__grammarDir,
			self.__grammarFile,
			self.__tokenPublisher,
			self.__fetchTraversableCategories,
			self.__askSparql,
		)
		self.waitForColdStartPermission()
		RtBiInterfaces.subscribeToIGraph(self, self.__onEvent)
		RtBiInterfaces.subscribeToIsomorphism(self, self.__onIsomorphism)
		RtBiInterfaces.subscribeToPredicates(self, self.__onPredicates)
		return

	def __onIsomorphism(self, msg: Msgs.RtBi.Isomorphism) -> None:
		# TODO: Report Isomorphism
		rawIsomorphism = loads(msg.isomorphism_json)
		isomorphism: dict[NodeId, NodeId] = {}
		for fromIdDictStr in rawIsomorphism:
			fromId = NodeId.fromJson(fromIdDictStr)
			toId = NodeId.fromJson(rawIsomorphism[fromIdDictStr])
			isomorphism[fromId] = toId
		self.__ba.updateTokensWithIsomorphism(isomorphism)
		return

	def __onEvent(self, msg: Msgs.RtBi.IGraph) -> None:
		iGraph = BehaviorIGraph.fromMsg(msg)
		if not self.__ba.initializedTokens: self.__ba.resetTokens(iGraph)
		# else: self.__ba.evaluate(iGraph)
		# After initialization, we need to evaluate the the tokens,
		# because their initial location may cause state transitions in the BA.
		# I'm assuming the else statement above was an optimization that causes an incorrect behavior.
		self.__ba.evaluate(iGraph)
		return

	def __onPredicates(self, predicateJsonStr: str) -> None:
		symMap = loads(predicateJsonStr)
		self.__ba.setSymbolicNameOfPredicate(symMap)
		return

	def onColdStartAllowed(self, payload: CustomPayload) -> None:
		if self.shouldRender: self.__ba.initFlask(self)
		self.publishColdStartDone({
			"predicates": self.__ba.predicates,
		})
		return

	def declareParameters(self) -> None:
		self.log(f"{self.get_fully_qualified_name()} is setting node parameters.")
		self.declare_parameter("grammar_dir", Parameter.Type.STRING)
		self.declare_parameter("grammar_file", Parameter.Type.STRING)
		self.declare_parameter("states", Parameter.Type.STRING_ARRAY)
		self.declare_parameter("transitions_from", Parameter.Type.STRING_ARRAY)
		self.declare_parameter("transitions_predicate", Parameter.Type.STRING_ARRAY)
		self.declare_parameter("transitions_to", Parameter.Type.STRING_ARRAY)
		self.declare_parameter("start", Parameter.Type.STRING)
		self.declare_parameter("accepting", Parameter.Type.STRING_ARRAY)
		return

	def parseParameters(self) -> None:
		self.log(f"{self.get_fully_qualified_name()} is parsing parameters.")
		self.__name = self.get_fully_qualified_name()
		self.__grammarDir = self.get_parameter("grammar_dir").get_parameter_value().string_value
		self.__grammarFile = self.get_parameter("grammar_file").get_parameter_value().string_value
		self.__states = list(self.get_parameter("states").get_parameter_value().string_array_value)
		frmList: list[str] = list(self.get_parameter("transitions_from").get_parameter_value().string_array_value)
		prdList: list[str] = list(self.get_parameter("transitions_predicate").get_parameter_value().string_array_value)
		toList: list[str] = list(self.get_parameter("transitions_to").get_parameter_value().string_array_value)
		for i in range(len(frmList)):
			frmState = frmList[i]
			toState = toList[i]
			prd = prdList[i]
			if frmState not in self.__transitions: self.__transitions[frmState] = {}
			self.__transitions[frmState][toState] = prd

		self.__start = self.get_parameter("start").get_parameter_value().string_value
		self.__accepting = list(self.get_parameter("accepting").get_parameter_value().string_array_value)
		return

	def render(self) -> None:
		if not self.shouldRender: return
		self.__ba.render()

	def __fetchTraversableCategories(self, tokenId: str, reqIris: list[str]) -> dict[str, list[str]]:
		if not reqIris:
			return {}
		req = Msgs.RtBiSrv.Tokens.Request()
		req.json_payload = dumps({
			"token_iri": f"target:{tokenId}",
			"req_iris": reqIris,
		})
		result: dict[str, list[str]] = {}
		def onResponse(_req: Msgs.RtBiSrv.Tokens.Request, res: Msgs.RtBiSrv.Tokens.Response) -> Msgs.RtBiSrv.Tokens.Response:
			Ros.Log("Received response for traversable categories request.", res.tokens)
			for tokenMsg in res.tokens:
				reqIri = tokenMsg.id
				result[reqIri] = [pred.value for pred in tokenMsg.attributes if pred.name == "target_category"]
			return res
		future = self.__tokensClient.call_async(req)
		event = threading.Event()
		future.add_done_callback(lambda _: event.set())
		event.wait()
		onResponse(req, future.result()) # pyright: ignore[reportArgumentType]
		return result

	def __askSparql(self, syntax: str, tokenIri: str) -> bool:
		req = Msgs.RtBiSrv.Tokens.Request()
		req.json_payload = dumps({"ask_query": syntax, "token_iri": tokenIri})
		result: bool = False
		def onResponse(_req: Msgs.RtBiSrv.Tokens.Request, res: Msgs.RtBiSrv.Tokens.Response) -> Msgs.RtBiSrv.Tokens.Response:
			nonlocal result
			for tokenMsg in res.tokens:
				if tokenMsg.id == "ask":
					for pred in tokenMsg.attributes:
						if pred.name == "ask_result":
							result = pred.value == Msgs.RtBi.Predicate.TRUE
			return res
		future = self.__tokensClient.call_async(req)
		event = threading.Event()
		future.add_done_callback(lambda _: event.set())
		event.wait()
		onResponse(req, future.result()) # pyright: ignore[reportArgumentType]
		return result

def main(args=None) -> None:
	import rclpy
	from rclpy.executors import MultiThreadedExecutor
	rclpy.init(args=args)
	node = BaNode()
	executor = MultiThreadedExecutor()
	executor.add_node(node)
	try:
		executor.spin()
	except KeyboardInterrupt:
		pass
	node.destroy_node()

if __name__ == "__main__":
	main()
