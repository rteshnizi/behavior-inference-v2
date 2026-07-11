from json import dumps
from math import isnan, nan
from pathlib import Path
from typing import Any, Literal

from ament_index_python.packages import get_package_share_directory

from rt_bi_commons.Base.ColdStartableNode import ColdStartable, ColdStartPayload
from rt_bi_commons.Base.DataDictionaryNode import DataDictionaryNode
from rt_bi_commons.RosParamParsers.AtomicParsers import StrParser
from rt_bi_commons.RosParamParsers.ParamParser import ParserBase
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_interfaces.srv import SpaceTime, Tokens
from rt_bi_runtime import package_name
from rt_bi_runtime.Model.FusekiInterface import FusekiInterface
from rt_bi_runtime.Model.SparqlTransformer import PredicateToQueryStr

QueryTemplates = Literal[
	"sparql_channel",
	"sparql_geometry",
	"sparql_intervals",
	"sparql_sets",
	"sparql_insert",
	"sparql_targets",
	"sparql_aggregation_operators",
	"sparql_attribute_kinds",
	"sparql_satisfies",
]
_Parameters = Literal[
	"fuseki_server",
	"rdf_dir",
	"rdf_store",
	"sparql_dir",
	"sparql_dir_operators",
	"placeholder_bind",
	"placeholder_ids",
	"placeholder_order",
	"placeholder_select",
	"placeholder_where",
	"placeholder_group_by",
	"placeholder_prop_uri",
	"placeholder_out_var",
	"placeholder_token_iri",
	"placeholder_restriction_blocks",
	"sparql_satisfies_ranged_block",
	"sparql_satisfies_discrete_block",
	"markup_select_fragment",
	"markup_group_by_fragment",
	"markup_where_fragment",
	"transition_grammar_dir",
	"transition_grammar_file",
] | QueryTemplates
# FIXME: Upload the rdf data at the startup via cold start.
class RdfStoreNode(ColdStartable, DataDictionaryNode[_Parameters]):
	def __init__(self, **kwArgs) -> None:
		parsers: dict[_Parameters, ParserBase[_Parameters, Any, Any]] = {
			"fuseki_server": StrParser[_Parameters](self, "fuseki_server"),
			"rdf_dir": StrParser[_Parameters](self, "rdf_dir"),
			"rdf_store": StrParser[_Parameters](self, "rdf_store"),
			"sparql_dir": StrParser[_Parameters](self, "sparql_dir"),
			"sparql_dir_operators": StrParser[_Parameters](self, "sparql_dir_operators"),
			"sparql_channel": StrParser[_Parameters](self, "sparql_channel"),
			"sparql_geometry": StrParser[_Parameters](self, "sparql_geometry"),
			"sparql_intervals": StrParser[_Parameters](self, "sparql_intervals"),
			"sparql_insert": StrParser[_Parameters](self, "sparql_insert"),
			"sparql_sets": StrParser[_Parameters](self, "sparql_sets"),
			"sparql_targets": StrParser[_Parameters](self, "sparql_targets"),
			"sparql_aggregation_operators": StrParser[_Parameters](self, "sparql_aggregation_operators"),
			"sparql_attribute_kinds": StrParser[_Parameters](self, "sparql_attribute_kinds"),
			"sparql_satisfies": StrParser[_Parameters](self, "sparql_satisfies"),
			"sparql_satisfies_ranged_block": StrParser[_Parameters](self, "sparql_satisfies_ranged_block"),
			"sparql_satisfies_discrete_block": StrParser[_Parameters](self, "sparql_satisfies_discrete_block"),
			"placeholder_bind": StrParser[_Parameters](self, "placeholder_bind"),
			"placeholder_ids": StrParser[_Parameters](self, "placeholder_ids"),
			"placeholder_order": StrParser[_Parameters](self, "placeholder_order"),
			"placeholder_where": StrParser[_Parameters](self, "placeholder_where"),
			"placeholder_select": StrParser[_Parameters](self, "placeholder_select"),
			"placeholder_group_by": StrParser[_Parameters](self, "placeholder_group_by"),
			"placeholder_prop_uri": StrParser[_Parameters](self, "placeholder_prop_uri"),
			"placeholder_out_var": StrParser[_Parameters](self, "placeholder_out_var"),
			"placeholder_token_iri": StrParser[_Parameters](self, "placeholder_token_iri"),
			"placeholder_restriction_blocks": StrParser[_Parameters](self, "placeholder_restriction_blocks"),
			"markup_select_fragment": StrParser[_Parameters](self, "markup_select_fragment"),
			"markup_group_by_fragment": StrParser[_Parameters](self, "markup_group_by_fragment"),
			"markup_where_fragment": StrParser[_Parameters](self, "markup_where_fragment"),
			"transition_grammar_dir": StrParser[_Parameters](self, "transition_grammar_dir"),
			"transition_grammar_file": StrParser[_Parameters](self, "transition_grammar_file"),
		}
		newKw = { "node_name": "dd_rdf", "loggingSeverity": Ros.LoggingSeverity.INFO, **kwArgs}
		DataDictionaryNode.__init__(self, parsers, **newKw)
		ColdStartable.__init__(self)
		self.__baseDir = get_package_share_directory(package_name)
		self.__httpInterface = FusekiInterface(self, self["fuseki_server"][0], self["rdf_store"][0])
		self.__predicateToIndex: dict[str, int] = {}
		# Maps property IRI -> aggregation operator IRI local-part (e.g., "minimum", "union").
		# Populated by __bootstrapOperatorCache on cold-start permission.
		self.__propToOperator: dict[str, str] = {}
		# Maps outer property IRI -> "discrete" or "ranged".
		# Populated by __bootstrapAttributeKindCache on cold-start permission.
		self.__attributeKindCache: dict[str, Literal["discrete", "ranged"]] = {}
		RtBiInterfaces.createSpaceTimeService(self, self.__onSpaceTimeRequest)
		RtBiInterfaces.createTargetsService(self, self.__onTargetsRequest)
		RtBiInterfaces.createSatisfiesService(self, self.__onSatisfiesRequest)
		RtBiInterfaces.subscribeToToken(self, self.__onToken)
		self.waitForColdStartPermission()

	def __joinList(self, l: list[str], separator: str) -> str:
		if len(l) == 0: return ""
		l = [s.strip() for s in l if s.strip() != ""]
		l = list(dict.fromkeys(l)) # Remove duplicates
		return separator.join(l)

	def __createFilterStatement(self, variables: list[str]) -> str:
		variables = list(filter(lambda s: s.startswith("?p_"), variables))
		condition = self.__joinList(variables, " || ")
		return f"FILTER ({condition})"

	def __templateParamToFileParam(self, param: QueryTemplates) -> str:
		if (
			param == "sparql_sets" or
			param == "sparql_channel" or
			param == "sparql_geometry" or
			param == "sparql_intervals" or
			param == "sparql_targets"
		):
			return self[param][0]
		raise RuntimeError(f"Unexpected template file parameter name: {param}")

	def __fillTemplate(self, templateParam: QueryTemplates, ids: list[str], whereClauses: list[str], variables: list[str], binds: list[str], orders: list[str], groupBy: list[str]) -> str:
		Ros.Log(f"Filling template for {templateParam}",
		[
			["ids:", ids],
			["where:", whereClauses],
			["variables:", variables],
			["binds:", binds],
			["orders:", orders],
			["groupBy:", groupBy],
		])
		fileName = self.__templateParamToFileParam(templateParam)
		sparql = Path(self.__baseDir, self["sparql_dir"][0], fileName).read_text()
		sparql = sparql.replace(self["placeholder_select"][0], self.__joinList(variables, " "))
		ids = [f"<{iri}>" for iri in ids]
		sparql = sparql.replace(self["placeholder_ids"][0], self.__joinList(ids, "\n\t\t"))
		sparql = sparql.replace(self["placeholder_where"][0], self.__joinList(whereClauses, "\n\t"))
		sparql = sparql.replace(self["placeholder_bind"][0], self.__joinList(binds, "\n\t"))
		sparql = sparql.replace(self["placeholder_order"][0], self.__joinList(orders, " "))
		sparql = sparql.replace(self["placeholder_group_by"][0], self.__joinList(groupBy, " "))
		return sparql

	def __onSpaceTimeRequest(self, req: SpaceTime.Request, res: SpaceTime.Response) -> SpaceTime.Response:
		payload = ColdStartPayload(req.json_payload)
		if req.query_name == "sets": res = self.__setQuery(payload, res)
		else: raise RuntimeError(f"Unexpected query name: {req.query_name}")
		return res

	def __setQuery(self, payload: ColdStartPayload, res: SpaceTime.Response) -> SpaceTime.Response:
		predicateMapping: dict[str, str] = {}
		ids: list[str] = []
		whereClauses: list[str] = []
		variables: list[str] = []
		binds: list[str] = []
		orders: list[str] = []
		sparqlXfmr = PredicateToQueryStr(self.__baseDir, self["transition_grammar_dir"][0], self["transition_grammar_file"][0])
		for predicate in payload.predicates:
			if predicate not in self.__predicateToIndex: self.__predicateToIndex[predicate] = len(self.__predicateToIndex)
			(extractedSelector, extractedVars, extractedBindings) = sparqlXfmr.transformPredicate(predicate, self.__predicateToIndex[predicate])
			if extractedVars == "" and extractedSelector == "": continue
			predicateMapping[predicate] = extractedVars
			variables.append(extractedVars)
			whereClauses.append(extractedSelector)
			binds.append(extractedBindings)
		# binds.append(self.__createFilterStatement(variables))
		sparql = self.__fillTemplate("sparql_sets", ids, whereClauses, variables, binds, orders, [])
		res.json_predicate_symbols = dumps(predicateMapping)
		msgsByTypeById = self.__httpInterface.fetchSets(sparql)
		setMsgs = self.__queryById("sparql_geometry", msgsByTypeById["static"] | msgsByTypeById["dynamic"])
		setMsgs |= self.__queryById("sparql_intervals", msgsByTypeById["dynamic"] | msgsByTypeById["temporal"])
		# setMsgs += self.__queryById(msgsByTypeAndId["affine"])
		Ros.ConcatMessageArray(res.sets, list(setMsgs.values()))
		return res

	def __extractFragment(self, content: str, markup: str) -> str:
		"""Returns the text strictly between two identical occurrences of `markup`.
		Raises if the markup pair is not found."""
		import re
		pattern = f"{re.escape(markup)}(.*){re.escape(markup)}"
		result = re.search(pattern, content, re.RegexFlag.DOTALL)
		if result is None:
			raise RuntimeError(f"Operator template missing markup pair: {markup!r}")
		return result.group(1).strip()

	def __substituteOperatorPlaceholders(self, fragment: str, propIri: str, outVar: str) -> str:
		fragment = fragment.replace(self["placeholder_prop_uri"][0], propIri)
		fragment = fragment.replace(self["placeholder_out_var"][0], outVar)
		return fragment

	__PROPERTY_PREFIX = "https://rezateshnizi.com/rt-bi-v2/property#"

	def __onTargetsRequest(self, req: Tokens.Request, res: Tokens.Response) -> Tokens.Response:
		payload = ColdStartPayload(req.json_payload)
		attributes: list[str] = list(payload.get("attributes", []))  # outer property short names
		ids: list[str] = list(payload.get("ids", []))                # full target IRIs
		selectFragments: list[str] = []
		groupByFragments: list[str] = []
		whereFragments: list[str] = []
		operatorsDir = self["sparql_dir_operators"][0]
		for shortName in attributes:
			propIri = f"{RdfStoreNode.__PROPERTY_PREFIX}{shortName}"
			if propIri not in self.__attributeKindCache:
				raise RuntimeError(f"Property {propIri!r} has no rdfs:range declared in the vocabulary.")
			kind = self.__attributeKindCache[propIri]
			pathStr = f"<{propIri}>"
			if kind == "discrete":
				opTemplate = Path(self.__baseDir, self["sparql_dir"][0], operatorsDir, "intersection.rq").read_text()
				selSec = self.__extractFragment(opTemplate, self["markup_select_fragment"][0])
				gbSec = self.__extractFragment(opTemplate, self["markup_group_by_fragment"][0])
				whSec = self.__extractFragment(opTemplate, self["markup_where_fragment"][0])
				if selSec: selectFragments.append(self.__substituteOperatorPlaceholders(selSec, pathStr, shortName))
				if gbSec: groupByFragments.append(self.__substituteOperatorPlaceholders(gbSec, pathStr, shortName))
				if whSec: whereFragments.append(self.__substituteOperatorPlaceholders(whSec, pathStr, shortName))
			elif kind == "ranged":
				for opFile, outVar in [("range_min.rq", f"{shortName}_min"), ("range_max.rq", f"{shortName}_max")]:
					opTemplate = Path(self.__baseDir, self["sparql_dir"][0], operatorsDir, opFile).read_text()
					selSec = self.__extractFragment(opTemplate, self["markup_select_fragment"][0])
					gbSec = self.__extractFragment(opTemplate, self["markup_group_by_fragment"][0])
					whSec = self.__extractFragment(opTemplate, self["markup_where_fragment"][0])
					if selSec: selectFragments.append(self.__substituteOperatorPlaceholders(selSec, pathStr, outVar))
					if gbSec: groupByFragments.append(self.__substituteOperatorPlaceholders(gbSec, pathStr, outVar))
					if whSec: whereFragments.append(self.__substituteOperatorPlaceholders(whSec, pathStr, outVar))
		sparql = self.__fillTemplate("sparql_targets", ids, whereFragments, selectFragments, [], [], groupByFragments)
		targetAttrs = self.__httpInterface.fetchTargets(sparql)
		Ros.Log(f"Targets query returned {len(targetAttrs)} target(s)", targetAttrs)
		for tokenIri, attrDict in targetAttrs.items():
			tokenMsg = Msgs.RtBi.Token()
			tokenMsg.id = tokenIri
			tokenMsg.stamp = Ros.Now(self).to_msg()
			for shortName in attributes:
				propIri = f"{RdfStoreNode.__PROPERTY_PREFIX}{shortName}"
				kind = self.__attributeKindCache[propIri]
				if kind == "discrete":
					rawVal = attrDict.get(shortName, "")
					values = [v.strip() for v in rawVal.split(",") if v.strip()] if rawVal else []
					attr = Msgs.RtBi.Attribute(name=shortName, kind="discrete")
					attr.discrete_values = values
					Ros.AppendMessage(tokenMsg.attributes, attr)
				elif kind == "ranged":
					minVal = attrDict.get(f"{shortName}_min", "")
					maxVal = attrDict.get(f"{shortName}_max", "")
					attr = Msgs.RtBi.Attribute(
						name=shortName, kind="ranged",
						range_min=float(minVal) if minVal else nan,
						range_max=float(maxVal) if maxVal else nan,
					)
					Ros.AppendMessage(tokenMsg.attributes, attr)
			Ros.AppendMessage(res.tokens, tokenMsg)
		return res

	def __queryById(self, templateParam: QueryTemplates, msgsById: dict[str, Msgs.RtBi.RegularSet]) -> dict[str, Msgs.RtBi.RegularSet]:
		if templateParam == "sparql_sets":
			raise RuntimeError(f"The template {templateParam} query doesn't have a placeholder for IDs")
		ids: list[str] = list(msgsById.keys())
		sparql = self.__fillTemplate(templateParam, ids, [], [], [], [], [])
		if templateParam == "sparql_geometry": return self.__httpInterface.fetchGeometryById(sparql, msgsById)
		if templateParam == "sparql_intervals": return self.__httpInterface.fetchIntervalsById(sparql, msgsById)
		raise RuntimeError(f"Unexpected template query param: {templateParam}")

	def __bootstrapOperatorCache(self) -> None:
		"""Populates self.__propToOperator by querying the vocabulary for all properties
		that declare property:aggregation_operator. The operator IRI's local-part doubles
		as the operator template filename basename (e.g., \"minimum\" -> operators/minimum.rq)."""
		fileName = self["sparql_aggregation_operators"][0]
		sparql = Path(self.__baseDir, self["sparql_dir"][0], fileName).read_text()
		self.__propToOperator = self.__httpInterface.fetchAggregationOperators(sparql)
		Ros.Log("Loaded aggregation operator cache", self.__propToOperator)

	def __bootstrapAttributeKindCache(self) -> None:
		"""Populates self.__attributeKindCache by querying the vocabulary for all outer
		attribute properties whose rdfs:range is class:DiscreteAttribute or class:RangedAttribute.
		The cache is a stable mapping used at dispatch time to select operator fragments."""
		fileName = self["sparql_attribute_kinds"][0]
		sparql = Path(self.__baseDir, self["sparql_dir"][0], fileName).read_text()
		self.__attributeKindCache = self.__httpInterface.fetchAttributeKinds(sparql)
		Ros.Log("Loaded attribute kind cache", self.__attributeKindCache)

	def onColdStartAllowed(self, payload: ColdStartPayload) -> None:
		self.__bootstrapOperatorCache()
		self.__bootstrapAttributeKindCache()
		self.publishColdStartDone()
		return

	def __buildRestrictionBlocks(self, restrictions: "list[Msgs.RtBi.Attribute]") -> str:
		"""Builds SPARQL FILTER blocks for each attribute restriction by filling
		the satisfies_ranged_block.rq / satisfies_discrete_block.rq templates."""
		rangedTemplate = Path(self.__baseDir, self["sparql_dir"][0], self["sparql_satisfies_ranged_block"][0]).read_text().rstrip()
		discreteTemplate = Path(self.__baseDir, self["sparql_dir"][0], self["sparql_satisfies_discrete_block"][0]).read_text().rstrip()
		blocks: list[str] = []
		for attr in restrictions:
			name = attr.name
			v = name.replace("_", "")
			if attr.kind == "ranged":
				rLo = attr.range_min
				rHi = attr.range_max
				overlap_parts: list[str] = []
				if not isnan(rHi):
					overlap_parts.append(f'COALESCE(?effLo_{v}, -1e308) <= "{rHi}"^^xsd:decimal')
				if not isnan(rLo):
					overlap_parts.append(f'COALESCE(?effHi_{v}, +1e308) >= "{rLo}"^^xsd:decimal')
				overlap_filter = " &&\n\t\t\t\t".join(overlap_parts) if overlap_parts else "true"
				block = (rangedTemplate
					.replace("# ATTR_NAME #", name)
					.replace("# ATTR_VAR #", v)
					.replace("# OVERLAP_FILTER #", overlap_filter)
				)
			else:
				allowed_iris = " ".join(self.__httpInterface.expandValueIri(name, val) for val in attr.discrete_values)
				block = (discreteTemplate
					.replace("# ATTR_NAME #", name)
					.replace("# ATTR_VAR #", v)
					.replace("# ALLOWED_IRIS #", allowed_iris)
				)
			blocks.append(block)
		return "\n".join(blocks)

	def __onSatisfiesRequest(self, req: "Msgs.RtBiSrv.Satisfies.Request", res: "Msgs.RtBiSrv.Satisfies.Response") -> "Msgs.RtBiSrv.Satisfies.Response":
		restrictions = Ros.AsList(req.restrictions, Msgs.RtBi.Attribute)
		restrictionBlocks = self.__buildRestrictionBlocks(restrictions)
		templatePath = Path(self.__baseDir, self["sparql_dir"][0], self["sparql_satisfies"][0])
		sparql = templatePath.read_text()
		sparql = sparql.replace(self["placeholder_token_iri"][0], req.token_id)
		sparql = sparql.replace(self["placeholder_restriction_blocks"][0], restrictionBlocks)
		res.satisfies = self.__httpInterface.askSatisfies(sparql)
		return res

	def __onToken(self, msg: Msgs.RtBi.Token) -> None:
		attrs = Ros.AsList(msg.attributes, Msgs.RtBi.Attribute)
		self.__httpInterface.insertToken(msg.id, msg.parent_id, attrs)

	def render(self) -> None:
		return super().render()

def main(args=None) -> None:
	return RdfStoreNode.Main(args)

if __name__ == "__main__":
	main()
