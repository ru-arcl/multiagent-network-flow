"""Exceptions and warnings raised by mppflow.

``Java*`` exceptions are raised only under ``compat="original"``, where they stand in
for the exception the original Java code throws at the same point. Each carries the Java
class name in ``java_class``. Every ``Java*`` class is defined here; the modules that
raise them (``planner``, ``problem``, ``solvers``, ``experiments``) re-export the ones
they used to define, so ``from mppflow.planner import JavaStackOverflowError`` and the like
keep working.

Hierarchy::

    MPPError
      Infeasible, TimeLimitReached, NotInOriginal, OriginalWouldHang
      JavaException                      (java.lang.RuntimeException)
        JavaNullPointerException
        JavaIndexOutOfBoundsException
          JavaArrayIndexOutOfBoundsException
        JavaArithmeticException, JavaNegativeArraySizeException
        JavaIllegalArgumentException
          JavaNumberFormatException
        JavaClassCastException, JavaArrayStoreException
        JavaClassNotFoundException
        JavaIOException
          JavaEOFException, JavaStreamCorruptedException, JavaInvalidClassException
        JavaStackOverflowError
        JavaGRBException
    InexactSolutionWarning (a RuntimeWarning)
"""

__all__ = [
    "MPPError",
    "Infeasible",
    "TimeLimitReached",
    "NotInOriginal",
    "OriginalWouldHang",
    "JavaException",
    "JavaNullPointerException",
    "JavaIndexOutOfBoundsException",
    "JavaArrayIndexOutOfBoundsException",
    "JavaArithmeticException",
    "JavaNegativeArraySizeException",
    "JavaIllegalArgumentException",
    "JavaNumberFormatException",
    "JavaClassCastException",
    "JavaArrayStoreException",
    "JavaClassNotFoundException",
    "JavaIOException",
    "JavaEOFException",
    "JavaStreamCorruptedException",
    "JavaInvalidClassException",
    "JavaStackOverflowError",
    "JavaGRBException",
    "InexactSolutionWarning",
]


class MPPError(Exception):
    """Base class for all mppflow errors."""


class Infeasible(MPPError):
    """The instance has no solution (within the given horizon bound, if any)."""


class TimeLimitReached(MPPError):
    """The time limit expired before a solution was found.

    Reserved: mppflow does not raise it. The planners and solvers report a time limit in
    their result (status ``"time_limit"`` without paths, or ``"feasible"``); callers who
    prefer an exception can raise this one on ``status == "time_limit"``."""


class NotInOriginal(MPPError):
    """The requested feature does not exist in the original code (compat="original")."""


class OriginalWouldHang(MPPError):
    """The original code enters an infinite loop on this input.

    Raised as soon as the loop is detected, instead of looping forever.
    """


class JavaException(MPPError):
    """An exception the original Java code throws on this input."""

    java_class = "java.lang.RuntimeException"


class JavaNullPointerException(JavaException):
    java_class = "java.lang.NullPointerException"


class JavaIndexOutOfBoundsException(JavaException):
    java_class = "java.lang.IndexOutOfBoundsException"


class JavaArrayIndexOutOfBoundsException(JavaIndexOutOfBoundsException):
    java_class = "java.lang.ArrayIndexOutOfBoundsException"


class JavaArithmeticException(JavaException):
    java_class = "java.lang.ArithmeticException"


class JavaNegativeArraySizeException(JavaException):
    java_class = "java.lang.NegativeArraySizeException"


class JavaIllegalArgumentException(JavaException):
    java_class = "java.lang.IllegalArgumentException"


class JavaNumberFormatException(JavaIllegalArgumentException):
    """``java.lang.NumberFormatException`` (``Integer.parseInt`` of ``Main.main``'s argument)."""

    java_class = "java.lang.NumberFormatException"


class JavaClassCastException(JavaException):
    """``java.lang.ClassCastException`` (raised by :func:`mppflow.problem.read_dat`)."""

    java_class = "java.lang.ClassCastException"


class JavaArrayStoreException(JavaException):
    """``java.lang.ArrayStoreException`` (raised by :func:`mppflow.problem.read_dat`)."""

    java_class = "java.lang.ArrayStoreException"


class JavaClassNotFoundException(JavaException):
    """``java.lang.ClassNotFoundException`` (a checked exception in Java; caught by the
    original's ``Problem.readFromFile``)."""

    java_class = "java.lang.ClassNotFoundException"


class JavaIOException(JavaException):
    """``java.io.IOException`` (checked in Java; the ``.dat`` reader's stream errors)."""

    java_class = "java.io.IOException"


class JavaEOFException(JavaIOException):
    """``java.io.EOFException``: the ``.dat`` stream ends early."""

    java_class = "java.io.EOFException"


class JavaStreamCorruptedException(JavaIOException):
    """``java.io.StreamCorruptedException``: raised internally by the ``.dat`` parser."""

    java_class = "java.io.StreamCorruptedException"


class JavaInvalidClassException(JavaIOException):
    """``java.io.InvalidClassException`` (caught by the original's ``readFromFile``)."""

    java_class = "java.io.InvalidClassException"


class JavaStackOverflowError(JavaException):
    """``java.lang.StackOverflowError``: the original recurses forever."""

    java_class = "java.lang.StackOverflowError"


class JavaGRBException(JavaException):
    """``gurobi.GRBException`` thrown by the original (e.g. reading ``ObjVal`` of a model
    without a solution)."""

    java_class = "com.gurobi.gurobi.GRBException"


class InexactSolutionWarning(RuntimeWarning):
    """``compat="original"`` path extraction met a value that is ``> 0.5`` but not exactly
    ``1.0``: the original's ``X == 1.0`` test fails there and the walk falls back to vertex 0,
    so the plan is invalid. Gurobi (the original's solver) normally returns exact binaries;
    other backends (HiGHS) often do not -- use ``backend="gurobi"`` in ``original`` mode."""
