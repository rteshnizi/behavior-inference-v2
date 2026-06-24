from typing import Any, Callable, Literal, Optional, TypedDict, cast

from rt_bi_commons.Base.TransitionParser import ParseTree, TransitionInterpreter, TransitionParser, TransitionTransformer, UnexpectedToken, v_args
from rt_bi_commons.Shared.Predicates import Predicates
from rt_bi_commons.Shared.Traversability import Attributes, DiscreteAttribute, RangedAttribute
from rt_bi_commons.Utils import Ros


class PredicateCollector(TransitionInterpreter):
	def __init__(self, simpleExpRebuildFn: Callable[[str, str, str], str]) -> None:
		super().__init__()
		self.__predicates: set[str] = set()
		self.__temporalPredicates: set[str] = set()
		self.__simpleExpRebuildFn: Callable[[str, str, str], str] = simpleExpRebuildFn

	@property
	def predicates(self) -> set[str]:
		return self.__predicates

	@property
	def temporalPredicates(self) -> set[str]:
		return self.__temporalPredicates

	def simple_expression(self, tree: ParseTree) -> str:
		children: list[str] = self.visit_children(tree)
		property_seq: str = cast(str, children[0])
		test: str = cast(str, children[1])
		value: str = cast(str, children[2])
		interpretationStr = self.__simpleExpRebuildFn(property_seq, test, value)
		self.__predicates.add(interpretationStr)
		return interpretationStr

	def property_seq(self, tree: ParseTree) -> str:
		children = super().property_seq(tree)
		return ".".join(children)

	def test(self, tree: ParseTree) -> str:
		self.visit_children(tree)
		if tree.children[0] == self.EQ(): return self.EQ()
		if tree.children[0] == self.NEQ(): return self.NEQ()
		if tree.children[0] == self.INCLUDES(): return self.INCLUDES()
		if tree.children[0] == self.IN(): return self.IN()
		if tree.children[0] == self.LTE(): return self.LTE()
		if tree.children[0] == self.GTE(): return self.GTE()
		raise UnexpectedToken(tree.children[0], {self.EQ, self.NEQ, self.INCLUDES, self.IN, self.LTE, self.GTE})

	def value(self, tree: ParseTree) -> str:
		children: list[str] = self.visit_children(tree)
		return children[0]

	def range_value(self, tree: ParseTree) -> str:
		"""Returns a canonical string like '[lo,hi]' for range_value tree nodes."""
		children: list[str] = self.visit_children(tree)
		return f"[{children[0]},{children[1]}]"

class TransitionEvaluator(TransitionTransformer):
	def __init__(self, predicateSymbolMap: dict[str, str], predicates: Predicates, simpleExpRebuildFn: Callable[[str, str, str], str]) -> None:
		super().__init__()
		self.__simpleExpRebuildFn = simpleExpRebuildFn
		self.__symbolMap = predicateSymbolMap
		"""Map from Transition Syntax to symbolic name."""
		self.__predicates = predicates
		"""Map from symbolic name to boolean value."""
		return

	def AND(self, _: str = "") -> Literal["and"]: return "and"
	def OR(self, _: str = "") -> Literal["or"]: return "or"
	def NOT(self, _: str = "") -> Literal["not"]: return "not"

	def expression(self, children: list[str]) -> str:
		fullStr = " ".join(children)
		return fullStr

	def connector(self, children: list[str]) -> str:
		return children[0]

	@v_args(tree=True)
	def simple_expression(self, tree: ParseTree) -> str:
		property_seq: str = cast(str, tree.children[0])
		test: str = cast(str, tree.children[1])
		value: str = cast(str, tree.children[2])
		transitionSyntax = self.__simpleExpRebuildFn(property_seq, test, value)
		if transitionSyntax not in self.__symbolMap: raise KeyError(f"{transitionSyntax} does not have a symbol.")
		sym = self.__symbolMap[transitionSyntax]
		# Not having a predicate is logically interpreted as False
		if sym not in self.__predicates: strVal = "False"
		else: strVal = str(self.__predicates[sym])
		return strVal

	def property_seq(self, children: list[str]) -> str:
		children = super().property_seq(children)
		return ".".join(children)

	def test(self, children: list[str]) -> str:
		return children[0]

	def value(self, children: list[str]) -> str:
		return children[0]

	def range_value(self, children: list[str]) -> str:
		"""Returns canonical range string like '[lo,hi]'."""
		return f"[{children[0]},{children[1]}]"

class TransitionStatement:
	def __init__(self, syntax: str, baseDir: str, grammarDir: str, grammarFileName: str) -> None:
		self.__str: str = syntax
		self.__symStr: str = syntax
		self.__parseTree = TransitionParser(baseDir, grammarDir, grammarFileName).parse(syntax)
		predCollector = PredicateCollector(self.__simpleExpRebuildFn)
		predCollector.visit(self.__parseTree)
		self.predicates: dict[str, str] = { p: "" for p in predCollector.predicates }
		"""Map from Transition Syntax to symbolic name."""
		# Target-attribute predicates: property_seq starts with "target."
		self.__targetAttributePredicates: set[str] = {
			p for p in predCollector.predicates if p.startswith("target.")
		}
		return

	def __simpleExpRebuildFn(self, property_seq: str, test: str, value: str) -> str:
		return f"{property_seq} {test} {value}"

	def __str__(self) -> str:
		return self.__str

	def __repr__(self) -> str:
		return self.__symStr

	def __hash__(self) -> int:
		return hash(self.__str)

	def __eq__(self, other: "TransitionStatement") -> bool:
		return self.__str == other.__str

	@property
	def targetAttributePredicates(self) -> set[str]:
		"""Predicates that reference target attributes (property_seq starts with 'target.')."""
		return self.__targetAttributePredicates

	def __contains__(self, syntax: str) -> bool:
		if syntax in self.predicates: return True
		return False

	def setPredicatesSymbol(self, syntax: str, symbol: str) -> None:
		if syntax == "" or symbol == "": return
		if syntax in self.predicates: self.predicates[syntax] = symbol
		self.__symStr = self.__symStr.replace(syntax, symbol)
		return

	def evaluate(self, predicates: Predicates, token_id: str = "", satisfiesFn: "Optional[Callable[[str, Any], bool]]" = None) -> bool:
		# Evaluate target-attribute predicates via checkSatisfiesFn if available
		if self.__targetAttributePredicates and token_id and satisfiesFn:
			Ros.Log(f"Evaluating TRANSITION PREDICATES for token {token_id}", self.__targetAttributePredicates)
			# Build per-attribute restrictions from target predicates
			# then AND them with the spatial result
			for pred in self.__targetAttributePredicates:
				# pred form: "target.<name> <op> <value>"
				parts = pred.split(" ")
				if len(parts) != 3: raise ValueError(f"Unexpected predicate format for target attribute predicate: {pred}")
				prop_seq, op, val = parts
				attrName = prop_seq.split(".", 1)[-1]
				restrictions = Attributes()
				if op == "INCLUDES":
					restrictions.items[attrName] = DiscreteAttribute(discrete_value=[val.strip('"')])
				elif op == "IN":
					# val like "[0.5,1.8]"
					inner = val.strip("[]").split(",")
					restrictions.items[attrName] = RangedAttribute(range_min=float(inner[0]), range_max=float(inner[1]))
				elif op == "<=":
					restrictions.items[attrName] = RangedAttribute(range_max=float(val))
				elif op == ">=":
					restrictions.items[attrName] = RangedAttribute(range_min=float(val))
				if not satisfiesFn(token_id, restrictions):
					return False
		# Evaluate spatial predicates
		# Remove target-attribute predicates from the predicate map for spatial evaluation
		spatialPredicateMap = {k: v for k, v in self.predicates.items() if k not in self.__targetAttributePredicates}
		evaluator = TransitionEvaluator(
			spatialPredicateMap,
			predicates,
			self.__simpleExpRebuildFn,
		)
		try:
			valStr = evaluator.transform(self.__parseTree) # pyright: ignore[reportArgumentType]
			val = eval(valStr)
			if isinstance(val, bool): return val
			else: raise ValueError(f"The evaluation result for {valStr} is not a boolean: {val}")
		except Exception as e:
			raise e

class Transition(TypedDict):
	label: str
	statement: TransitionStatement
