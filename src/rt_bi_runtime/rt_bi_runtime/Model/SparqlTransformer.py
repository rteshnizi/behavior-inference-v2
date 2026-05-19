from typing import Any, Callable, Literal, cast

from rt_bi_commons.Base.TransitionParser import Discard, Token, TransitionParser, TransitionTransformer, Tree, UnexpectedToken
from rt_bi_commons.Utils import Ros


class PredicateToWhere(TransitionTransformer):
	def __init__(self, toVarName: Callable[[list[str]], str]) -> None:
		self.__toVarName = toVarName
		return

	def NOT(self, _: Any) -> Any: return Discard
	def connector(self, _: Any) -> Any: return Discard
	def value(self, _: Any) -> Any: return Discard
	def test(self, _: Any) -> Any: return Discard

	def simple_expression(self, subExpressions: list[list[str]]) -> str:
		# OPTIONAL { ?regularSetId property:material/property:name ?name . }
		if len(subExpressions) != 1: raise UnexpectedToken(subExpressions, {"1 strings: variable."})
		variables = subExpressions[0]
		# ["material", "name"]
		titleCase = self.__toVarName(variables)
		variables = [f"property:{v}" for v in variables]
		propSeq = "/".join(variables)
		# property:material/property:name
		where = f"OPTIONAL {{ ?regularSetId {propSeq} {titleCase} }}"
		return where

	def property_seq(self, variables: list[str]) -> list[str]:
		return variables

class PredicateToBind(TransitionTransformer):
	def __init__(self, index: int, toVarName: Callable[[list[str]], str]) -> None:
		super().__init__()
		self.__toVarName = toVarName
		self.propositionalSymbol = f"?p_{index}"
		""" String of the form `?p_i` where `i` is the index of the proportional symbol. """
		return

	def EQ(self, _: Any) -> Literal["="]: return "="
	def connector(self, _: Token) -> Any: raise UnexpectedToken(_, {"No connectors are expected in a single predicate."})
	def value(self, v: list[str]) -> str: return v[0]
	def test(self, t: list[str]) -> str: return t[0]

	def simple_expression(self, subExpressions: list[str]) -> str:
		if len(subExpressions) != 3: raise UnexpectedToken(subExpressions, {"3 strings: variable, test, value"})
		return f"BIND ({subExpressions[0]} {subExpressions[1]} {subExpressions[2]} AS {self.propositionalSymbol})"

	def property_seq(self, variables: list[str]) -> str:
		return self.__toVarName(variables)

class PredicateToQueryStr:
	def __init__(
		self,
		baseDir: str,
		transitionGrammarDir: str,
		transitionGrammarFileName: str,
	) -> None:
		super().__init__()
		self.__transitionParser = TransitionParser(baseDir, transitionGrammarDir, transitionGrammarFileName)
		return

	def __propSeqToVariableName(self, variables: list[str]) -> str:
		""" Converts `["material", "name"]` to `"?MaterialName"`. """
		titleCase = ''.join([s.title() for s in variables])
		return f"?{titleCase}"

	def __toBind(self, parseTree: Tree[str], index: int) -> tuple[str, str]:
		bindXfmr = PredicateToBind(index, self.__propSeqToVariableName)
		bindStatement = cast(str, bindXfmr.transform(parseTree))
		if bindStatement == "": return ("", "")
		return (bindStatement, bindXfmr.propositionalSymbol)

	def __toWhere(self, parseTree: Tree[str]) -> str:
		whereXfmr = PredicateToWhere(self.__propSeqToVariableName)
		whereClause = whereXfmr.transform(parseTree)
		return whereClause

	def transformPredicate(self, predicate: str, index: int) -> tuple[str, str, str]:
		# For predicates, we add the bound boolean variables rather than the sparql variable
		parseTree = self.__transitionParser.parse(predicate)
		parseTree = cast(Tree[str], parseTree)
		whereClause = self.__toWhere(parseTree)
		(varBindings, variables) = self.__toBind(parseTree, index)
		return (whereClause, variables, varBindings)

class TargetAttributeEvaluator(TransitionTransformer):
	"""Transforms a transition parse tree into a SPARQL WHERE-body fragment.
	Only `target.*` simple expressions produce SPARQL patterns; non-target
	expressions return None and are silently dropped.
	"""

	def NOT(self, _: Any) -> Any: return Discard
	def AND(self, _: Any) -> str: return "AND"
	def OR(self, _: Any) -> str: return "OR"

	def property_seq(self, children: list[str]) -> list[str]:
		return super().property_seq(children)

	def test(self, children: list[str]) -> str:
		return children[0]

	def value(self, children: list[str]) -> str:
		v = str(children[0])
		if v.startswith('"') and v.endswith('"'):
			return v[1:-1]
		return v

	def simple_expression(self, children: list) -> "str | None":
		prop_seq: list[str] = children[0]
		test_op: str = children[1]
		val: str = children[2]
		if prop_seq[0] != "target":
			return None
		prop_path = "/".join(f"property:{p}" for p in prop_seq[1:])
		if test_op == "==":
			return f"?token {prop_path} {val} ."
		return f"FILTER NOT EXISTS {{ ?token {prop_path} {val} . }}"

	def negated_expression(self, children: list) -> "str | None":
		patterns = [c for c in children if c is not None]
		if not patterns:
			return None
		return f"FILTER NOT EXISTS {{\n\t\t{patterns[0]}\n\t}}"

	def expression(self, children: list) -> "str | None":
		connector: "str | None" = None
		patterns: list[str] = []
		for c in children:
			if c in ("AND", "OR"):
				connector = c
			elif c is not None:
				patterns.append(str(c))
		if not patterns:
			return None
		if connector is None or len(patterns) == 1:
			return "\n\t".join(patterns)
		left, right = patterns[0], patterns[1]
		if connector == "AND":
			return f"{left}\n\t{right}"
		return f"{{\n\t\t{left}\n\t}} UNION {{\n\t\t{right}\n\t}}"

	def connector(self, children: list[str]) -> str:
		return children[0]

class TargetAskWhereBuilder:
	"""Parses a transition syntax string and produces a SPARQL WHERE-body fragment
	for use in an ASK query (via predicate_ask.rq)."""

	def __init__(self, baseDir: str, transitionGrammarDir: str, transitionGrammarFileName: str) -> None:
		super().__init__()
		self.__transitionParser = TransitionParser(baseDir, transitionGrammarDir, transitionGrammarFileName)

	def buildBody(self, syntax: str) -> str:
		parseTree = cast(Tree[str], self.__transitionParser.parse(syntax))
		body = TargetAttributeEvaluator().transform(parseTree)  # pyright: ignore[reportArgumentType]
		return str(body) if body is not None else ""
