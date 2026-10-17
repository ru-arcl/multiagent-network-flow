"""Linear and mixed-integer programming: a neutral model and three backends.

* :class:`LinearModel`, :class:`Solution` (:mod:`mppflow.lp.linear`)
* ``backend="highs"``: scipy's HiGHS (:mod:`mppflow.lp.highs`), the default
* ``backend="gurobi"``: gurobipy (:mod:`mppflow.lp.gurobi`), reproduces the original's
  solutions exactly (same version, same ``Threads``)
* ``backend="builtin"``: pure-Python simplex and branch and bound (:mod:`mppflow.lp.builtin`),
  small models only; the same algorithm as ``docs/js/lp.js``

This package is new (the original calls Gurobi directly); it has no ``compat`` switch.
"""

from __future__ import annotations

from .linear import INF, STATUSES, LinearModel, Solution, normalize_sense, relative_gap

BACKENDS = ("highs", "gurobi", "builtin")

__all__ = ["INF", "STATUSES", "BACKENDS", "LinearModel", "Solution", "solve",
           "available_backends", "normalize_sense", "relative_gap"]


def available_backends() -> list[str]:
    """The backends usable here, in preference order (``builtin`` is always available).

    ``gurobi`` is listed when gurobipy imports; a license is not checked.
    """
    out = []
    try:
        import scipy.optimize

        if hasattr(scipy.optimize, "milp"):
            out.append("highs")
    except ImportError:  # pragma: no cover
        pass
    from . import gurobi

    if gurobi.available():
        out.append("gurobi")
    out.append("builtin")
    return out


def solve(
    model: LinearModel,
    backend: str = "highs",
    time_limit: float | None = None,
    mip_gap: float | None = None,
    params: dict | None = None,
    relax: bool = False,
) -> Solution:
    """Solve ``model`` with the chosen backend.

    Args:
        model: the model.
        backend: ``"highs"`` (default), ``"gurobi"`` or ``"builtin"``.
        time_limit: seconds (``None``: unlimited). For ``builtin`` this is checked between
            branch-and-bound nodes only.
        mip_gap: relative MIP gap (``None``: the backend's default, 1e-4 for HiGHS and
            Gurobi, 0 for ``builtin``).
        params: backend-specific options: Gurobi parameters for ``gurobi`` (e.g.
            ``{"Threads": 1}``), ``milp`` options for ``highs``, and ``node_limit`` /
            ``work_limit`` for ``builtin``. On every backend ``time_limit`` and ``mip_gap``
            take precedence over the same settings in ``params``.
        relax: solve the LP relaxation instead.
    """
    if backend == "highs":
        from . import highs

        return highs.solve(model, time_limit=time_limit, mip_gap=mip_gap, params=params,
                           relax=relax)
    if backend == "gurobi":
        from . import gurobi

        return gurobi.solve(model, time_limit=time_limit, mip_gap=mip_gap, params=params,
                            relax=relax)
    if backend == "builtin":
        from . import builtin

        return builtin.solve(model.relax() if relax else model, time_limit=time_limit,
                             mip_gap=mip_gap, params=params)
    raise ValueError(f"unknown backend {backend!r}; choose one of {BACKENDS}")
