# ADR 0002 — Error handling

## Status

Accepted.

## Context

A manipulation library meets two kinds of failure: programming or configuration errors (wrong vector
sizes, non-finite values, inconsistent models, invalid options) and expected algorithmic outcomes (an
IK solve that does not converge, a planner that exhausts its budget, an unreachable goal). Treating
both with exceptions forces callers to use exceptions for control flow; treating both with status
codes makes misuse easy to ignore.

## Decision

* Invalid input or configuration throws `mc::InvalidArgument` (derived from `std::invalid_argument`)
  at the API boundary, with a message naming the argument.
* Expected outcomes are values: `IKResult` (`IKStatus`), `PlannerResult` (`PlannerStatus` and
  statistics), `PoseGoalPlanningResult` (`success` and `message`).
* No public function returns NaN or infinity for finite, valid input; numerical breakdowns that would
  produce them (non-positive-definite mass matrix, divergent simulation) raise exceptions instead.
* The library never prints; applications decide what to log.

## Consequences

* Normal control flow (retrying IK, trying another planner) needs no exceptions.
* Hot loops validate sizes cheaply and build messages only on failure.
* Tests check both sides: invalid input throws, and failure statuses carry finite, in-limit data.
