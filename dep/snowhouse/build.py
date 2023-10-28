from build.c import cxxlibrary

cxxlibrary(
    name="snowhouse",
    hdrs={
        "snowhouse/snowhouse.h": "./include/snowhouse/snowhouse.h",
        "snowhouse/assert.h": "./include/snowhouse/assert.h",
        "snowhouse/fluent/fluent.h": "./include/snowhouse/fluent/fluent.h",
        "snowhouse/fluent/constraintadapter.h": "./include/snowhouse/fluent/constraintadapter.h",
        "snowhouse/fluent/constraintlist.h": "./include/snowhouse/fluent/constraintlist.h",
        "snowhouse/fluent/operators/andoperator.h": "./include/snowhouse/fluent/operators/andoperator.h",
        "snowhouse/fluent/operators/invalidexpressionexception.h": "./include/snowhouse/fluent/operators/invalidexpressionexception.h",
        "snowhouse/fluent/operators/collections/collectionoperator.h": "./include/snowhouse/fluent/operators/collections/collectionoperator.h",
        "snowhouse/fluent/operators/collections/collectionconstraintevaluator.h": "./include/snowhouse/fluent/operators/collections/collectionconstraintevaluator.h",
        "snowhouse/fluent/operators/collections/atleastoperator.h": "./include/snowhouse/fluent/operators/collections/atleastoperator.h",
        "snowhouse/fluent/operators/collections/noneoperator.h": "./include/snowhouse/fluent/operators/collections/noneoperator.h",
        "snowhouse/fluent/operators/collections/atmostoperator.h": "./include/snowhouse/fluent/operators/collections/atmostoperator.h",
        "snowhouse/fluent/operators/collections/alloperator.h": "./include/snowhouse/fluent/operators/collections/alloperator.h",
        "snowhouse/fluent/operators/collections/exactlyoperator.h": "./include/snowhouse/fluent/operators/collections/exactlyoperator.h",
        "snowhouse/fluent/operators/notoperator.h": "./include/snowhouse/fluent/operators/notoperator.h",
        "snowhouse/fluent/operators/constraintoperator.h": "./include/snowhouse/fluent/operators/constraintoperator.h",
        "snowhouse/fluent/operators/oroperator.h": "./include/snowhouse/fluent/operators/oroperator.h",
        "snowhouse/fluent/expressionbuilder.h": "./include/snowhouse/fluent/expressionbuilder.h",
        "snowhouse/assertionexception.h": "./include/snowhouse/assertionexception.h",
        "snowhouse/exceptions.h": "./include/snowhouse/exceptions.h",
        "snowhouse/stringizers.h": "./include/snowhouse/stringizers.h",
        "snowhouse/macros.h": "./include/snowhouse/macros.h",
        "snowhouse/constraints/equalscontainerconstraint.h": "./include/snowhouse/constraints/equalscontainerconstraint.h",
        "snowhouse/constraints/islessthanorequaltoconstraint.h": "./include/snowhouse/constraints/islessthanorequaltoconstraint.h",
        "snowhouse/constraints/equalsconstraint.h": "./include/snowhouse/constraints/equalsconstraint.h",
        "snowhouse/constraints/isgreaterthanconstraint.h": "./include/snowhouse/constraints/isgreaterthanconstraint.h",
        "snowhouse/constraints/fulfillsconstraint.h": "./include/snowhouse/constraints/fulfillsconstraint.h",
        "snowhouse/constraints/endswithconstraint.h": "./include/snowhouse/constraints/endswithconstraint.h",
        "snowhouse/constraints/constraints.h": "./include/snowhouse/constraints/constraints.h",
        "snowhouse/constraints/haslengthconstraint.h": "./include/snowhouse/constraints/haslengthconstraint.h",
        "snowhouse/constraints/startswithconstraint.h": "./include/snowhouse/constraints/startswithconstraint.h",
        "snowhouse/constraints/equalswithdeltaconstraint.h": "./include/snowhouse/constraints/equalswithdeltaconstraint.h",
        "snowhouse/constraints/isgreaterthanorequaltoconstraint.h": "./include/snowhouse/constraints/isgreaterthanorequaltoconstraint.h",
        "snowhouse/constraints/containsconstraint.h": "./include/snowhouse/constraints/containsconstraint.h",
        "snowhouse/constraints/islessthanconstraint.h": "./include/snowhouse/constraints/islessthanconstraint.h",
        "snowhouse/constraints/isemptyconstraint.h": "./include/snowhouse/constraints/isemptyconstraint.h",
        "snowhouse/constraints/expressions/andexpression.h": "./include/snowhouse/constraints/expressions/andexpression.h",
        "snowhouse/constraints/expressions/orexpression.h": "./include/snowhouse/constraints/expressions/orexpression.h",
        "snowhouse/constraints/expressions/expression_fwd.h": "./include/snowhouse/constraints/expressions/expression_fwd.h",
        "snowhouse/constraints/expressions/notexpression.h": "./include/snowhouse/constraints/expressions/notexpression.h",
        "snowhouse/constraints/expressions/expression.h": "./include/snowhouse/constraints/expressions/expression.h",
        "snowhouse/stringize.h": "./include/snowhouse/stringize.h",
    },
)
