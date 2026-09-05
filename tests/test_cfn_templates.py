"""Lint the CloudFormation templates the way the app code is linted.

This exists because of one deploy failure that cost a full build-and-push
cycle to diagnose. `cloudformation/serverless.yaml` had:

    OriginSSLProtocols: {Quantity: 1, Items: [TLSv1.2]}

which is the shape the CloudFront *API* takes -- correct in an
`aws cloudfront create-distribution` call, and wrong in CloudFormation,
which wants a plain list. The template passed `validate-template`, and the
change set then failed with:

    The following hook(s)/validation failed:
    [AWS::EarlyValidation::PropertyValidation]

That message names neither the resource nor the property, `describe-events`
returned nothing, and `list-hook-results` was empty. cfn-lint located it in
one run: "E3012 line 294: {'Quantity': 1, 'Items': [...]} is not of type
'array'".

Skips rather than fails without cfn-lint installed, matching how the
Playwright UI tests behave without a browser -- the rest of the suite still
runs on a machine that has not installed it.
"""

import pytest

from tests.conftest import REPO_ROOT

CFN = REPO_ROOT / "cloudformation"

api = pytest.importorskip(
    "cfnlint.api",
    reason="cfn-lint not installed -- pip install cfn-lint to run template linting",
)


def blocking(template):
    """Errors that would actually fail a deploy in the region this project
    uses. Partition-availability notes for cn-* and us-gov-* are filtered:
    every stack here is us-east-2 and cfn-lint checks all regions by default,
    so those are noise that would make the test useless."""
    matches = api.lint_all(template.read_text())
    return [m for m in matches
            if m.rule.id.startswith("E") and "does not exist in" not in m.message]


@pytest.mark.parametrize("name", sorted(p.name for p in CFN.glob("*.yaml")))
def test_template_has_no_blocking_lint_errors(name):
    errors = blocking(CFN / name)
    assert not errors, "\n".join(
        f"{m.rule.id} line {m.linenumber}: {m.message}" for m in errors)
