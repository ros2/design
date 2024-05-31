#! /usr/bin/env python

# PoC Ros2Names parsing using EBNF and Grako (install from pypi)
# Tested with grako 3.14.0
#


## TODO:
# maximum length + example

## ambiguous texts
# "balanced" curly braces may imply nesting?
# url scheme only 'rostopic' and 'rosservice' allowed?


import re
import grako


POSITIVES_QUALIFIED = [
    "/x", "/foo", "/abc123", "/_foo", "/Foo", "/BAR",
    "/foo/bar", "/x/y/z",
    "/_foo/bar", "/_f_o_o/bar",
    "rosservice:///foo", "rostopic:///foo/bar"
    ]

POSITIVES_UNQUALIFIED = [
    "x", "foo", "abc123", "_foo", "Foo", "BAR",
    "x/y/z",
    "foo/bar",
     "{foo}", "{foo}_bar", "foo/{ping}/bar",
     "foo{x1}bar{x2}baz", "{x1}bar{x2}baz{top}",
     "foo{x1}/bar/{x2}baz", "{x1}/bar/{x2}",
     "~", "~/foo", "~/foo/x/_123",
     "_foo/bar", "_f_o_o/bar",
     "rosservice://foo", "rostopic://foo/bar", "rostopic://~/_bar/foo{x}",
     "~/{foo}",
     "foo{bar__baz}", "foo{_baz_}"
    ]

POSITIVES = POSITIVES_QUALIFIED + POSITIVES_UNQUALIFIED

NEGATIVES_QUALIFIED = [
    # empty
    "/", "//",
    # starts with numeric
    "/foo/123bar", "/foo/123/bar",
    # ends with underscore
    "/foo/_/bar", "/foo_/bar", "/_/bar", "_", "_/_bar"
    # contains illegal char
    "foo bar", "foo^bar", "foo!bar",
    # missing double slashes
    "rosservice:/foo", "rostopic:/foo/bar", "rostopic:foo"
    ## bad tilde
    "~~", "/foo/~/",
    ]

NEGATIVES_UNQUALIFIED = [
    # empty
    "",
    "123abc", "123",
    "__foo", "foo__bar", "foo__",
    "foo_", "foo_/bar",  "foo/_/bar",
    "foo//bar",
    "foo/",
    "{", "}", "}{", "foo{}", "foo{bar{baz}}", "foo{}bar{}baz"
    "~foo", "foo~", "foo~/bar", "foo/~bar", "/~", "foo/~/bar"
    ]

NEGATIVES = NEGATIVES_QUALIFIED + NEGATIVES_UNQUALIFIED



## Short EBNF Grako introduction
# [] optional element
# {} repetition (zero or more)
# repetitions are greedy
# (* *) comment
# | alternative  (order matters (for grako at least)), first are tried first)
# '' token
# / / regex
GRAMMAR = """

uriname
    =
    uriname_qualified
    |
    [ urischeme ] pathname $
    ;

uriname_qualified
    =
    [ urischeme ] pathname_qualified $
    ;

urischeme
    =
    namecharp '://'
    ;

pathname
    =
    ['/' | '~/'] nametoken '/' { nametoken '/' } nametoken
    |
    ['/' | '~/'] singlename
    |
    '~'
    ;

pathname_qualified
    =
    '/' nametoken_qualified '/' { nametoken_qualified '/' } nametoken_qualified
    |
    '/' singlename_qualified
    ;

(* must not start with digit, must not finish with _ **)
nametoken
    =
    substitution { substitution } [ nametoken_qualified ]
    |
    nametoken_qualified
    ;

(* must not start with digit finish with _ *)
nametoken_qualified
    =
    /[_A-Za-z][_A-Za-z0-9]*[A-Za-z0-9]/
    |
    /[A-Za-z]/
    ;

(* must not start with digit, nor finish with _ *)
singlename
    =
    [ /[_A-Za-z][_A-Za-z0-9]*/ ] substitution { substitution } [ /[_A-Za-z0-9]*[A-Za-z0-9]/ ]
    |
    singlename_qualified
    ;

(* must not start with digit, nor finish with _ *)
singlename_qualified
    =
    /[_A-Za-z][_A-Za-z0-9]*[A-Za-z0-9]/
    |
    /[A-Za-z]/
    ;

substitution
    =
    namecharx '{' namecharp '}'
    ;

namecharp
    =
    /[_A-Za-z0-9]+/
    ;

namecharx
    =
    /[_A-Za-z0-9]*/
    ;

"""

MODEL = grako.genmodel("model", GRAMMAR)


def parse_qualified(s):
    ast = MODEL.parse(s, "uriname_qualified")
    if '__' in s:
        raise SyntaxError("Must not contain consecutive underlines: '" + s + "'")
    return ast

def resolve(s):
    return re.sub(r'\{[a-zA-Z0-9_]+\}', 'xxx', s)

def parse(s):
    ast = MODEL.parse(s, "uriname")
    if '__' in resolve(s):
        raise SyntaxError("Must not contain consecutive underlines: '" + resolve(s) + "'")
    return ast


def test_main():
    for s in POSITIVES:
        print("ACCEPTED: %s \t as %s " % (s, parse(s)))
    for s in POSITIVES_QUALIFIED:
        print("ACCEPTED QUALIFIED: %s \t as %s " % (s, parse_qualified(s)))

    print('=' * 40)

    for s in NEGATIVES:
        try:
            raise AssertionError("ERROR, SHOULD REFUSE: %s \t\t but was %s" % (s, parse(s)))
        except (grako.exceptions.FailedParse, SyntaxError):
            print("REJECTED: " + s)

    for s in NEGATIVES_QUALIFIED + POSITIVES_UNQUALIFIED:
        try:
            raise AssertionError("ERROR, SHOULD REFUSE: %s \t\t but was %s" % (s, parse_qualified(s)))
        except (grako.exceptions.FailedParse, SyntaxError):
            print("REJECTED QUALIFIED: " + s)


if __name__ == '__main__':
    test_main()
