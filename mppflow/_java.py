"""Emulation of the Java behaviour the original code depends on.

* :class:`JavaRandom` -- ``java.util.Random`` (48-bit LCG), bit for bit.
* :class:`MathRandom` -- the generator behind ``java.lang.Math.random()``.
* :class:`JavaIntHashSet` -- iteration order of ``java.util.HashSet<Integer>`` (JDK 8+),
  treeified bins included.
* :func:`i32` -- Java ``int`` wrap-around.
* :func:`java_double_str` -- ``Double.toString`` for the values the original prints.
"""
from __future__ import annotations

import math

_MULT = 0x5DEECE66D
_ADD = 0xB
_MASK = (1 << 48) - 1


def i32(x: int) -> int:
    """Wrap an integer to a signed 32-bit Java ``int``."""
    x &= 0xFFFFFFFF
    return x - (1 << 32) if x >= (1 << 31) else x


def i64(x: int) -> int:
    """Wrap an integer to a signed 64-bit Java ``long``."""
    x &= 0xFFFFFFFFFFFFFFFF
    return x - (1 << 64) if x >= (1 << 63) else x


class JavaRandom:
    """``java.util.Random``: same seed, same stream."""

    def __init__(self, seed: int = 0):
        self.set_seed(seed)

    def set_seed(self, seed: int) -> None:
        self._seed = (seed ^ _MULT) & _MASK

    def next_bits(self, bits: int) -> int:
        self._seed = (self._seed * _MULT + _ADD) & _MASK
        return i32(self._seed >> (48 - bits))

    def next_double(self) -> float:
        return ((self.next_bits(26) << 27) + self.next_bits(27)) * (1.0 / (1 << 53))

    def next_int(self, bound: int | None = None) -> int:
        if bound is None:
            return self.next_bits(32)
        if bound <= 0:
            raise ValueError("bound must be positive")
        r = self.next_bits(31)
        m = bound - 1
        if bound & m == 0:
            return (bound * r) >> 31
        u = r
        while True:
            r = u % bound
            if u - r + m < (1 << 31):
                return r
            u = self.next_bits(31)

    def state(self) -> int:
        """The internal 48-bit state (for tests)."""
        return self._seed


class MathRandom(JavaRandom):
    """Stand-in for the JVM-wide ``Math.random()`` generator.

    ``MathRandom(seed).random()`` yields the same values as ``Math.random()`` after the
    original JVM's generator was seeded with ``setSeed(seed)``. ``draws`` counts the
    values handed out, which tests use to check consumption order.
    """

    def __init__(self, seed: int = 0):
        super().__init__(seed)
        self.draws = 0

    def random(self) -> float:
        self.draws += 1
        return self.next_double()


_default_rng = MathRandom(0)


def default_rng() -> MathRandom:
    """The module-level stream used when a function receives ``rng=None``."""
    return _default_rng


def seed_default_rng(seed: int) -> None:
    global _default_rng
    _default_rng = MathRandom(seed)


def resolve_rng(rng: MathRandom | None) -> MathRandom:
    return _default_rng if rng is None else rng


class _TreeNode:
    """A node of a treeified ``HashMap`` bin (``HashMap.TreeNode``): red-black links only;
    the bin's ``next`` chain is the bin list itself."""

    __slots__ = ("key", "hash", "left", "right", "parent", "red")

    def __init__(self, key: int, h: int):
        self.key = key
        self.hash = h
        self.left = self.right = self.parent = None
        self.red = False


def _tn_dir(h: int, k: int, p: _TreeNode) -> int:
    # Signed hash comparison, then compareTo (Integer/Long keys are Comparable and
    # distinct keys never compare equal, so tieBreakOrder is never reached).
    if p.hash > h:
        return -1
    if p.hash < h:
        return 1
    return -1 if k < p.key else 1


def _rotate_left(root, p):
    if p is not None and p.right is not None:
        r = p.right
        rl = p.right = r.left
        if rl is not None:
            rl.parent = p
        pp = r.parent = p.parent
        if pp is None:
            root = r
            r.red = False
        elif pp.left is p:
            pp.left = r
        else:
            pp.right = r
        r.left = p
        p.parent = r
    return root


def _rotate_right(root, p):
    if p is not None and p.left is not None:
        l = p.left
        lr = p.left = l.right
        if lr is not None:
            lr.parent = p
        pp = l.parent = p.parent
        if pp is None:
            root = l
            l.red = False
        elif pp.right is p:
            pp.right = l
        else:
            pp.left = l
        l.right = p
        p.parent = l
    return root


def _balance_insertion(root, x):
    x.red = True
    while True:
        xp = x.parent
        if xp is None:
            x.red = False
            return x
        if not xp.red or xp.parent is None:
            return root
        xpp = xp.parent
        xppl = xpp.left
        if xp is xppl:
            xppr = xpp.right
            if xppr is not None and xppr.red:
                xppr.red = False
                xp.red = False
                xpp.red = True
                x = xpp
            else:
                if x is xp.right:
                    x = xp
                    root = _rotate_left(root, x)
                    xp = x.parent
                    xpp = None if xp is None else xp.parent
                if xp is not None:
                    xp.red = False
                    if xpp is not None:
                        xpp.red = True
                        root = _rotate_right(root, xpp)
        else:
            if xppl is not None and xppl.red:
                xppl.red = False
                xp.red = False
                xpp.red = True
                x = xpp
            else:
                if x is xp.left:
                    x = xp
                    root = _rotate_right(root, x)
                    xp = x.parent
                    xpp = None if xp is None else xp.parent
                if xp is not None:
                    xp.red = False
                    if xpp is not None:
                        xpp.red = True
                        root = _rotate_left(root, xpp)


def _balance_deletion(root, x):
    while True:
        if x is None or x is root:
            return root
        xp = x.parent
        if xp is None:
            x.red = False
            return x
        if x.red:
            x.red = False
            return root
        xpl = xp.left
        if xpl is x:
            xpr = xp.right
            if xpr is not None and xpr.red:
                xpr.red = False
                xp.red = True
                root = _rotate_left(root, xp)
                xp = x.parent
                xpr = None if xp is None else xp.right
            if xpr is None:
                x = xp
            else:
                sl, sr = xpr.left, xpr.right
                if (sr is None or not sr.red) and (sl is None or not sl.red):
                    xpr.red = True
                    x = xp
                else:
                    if sr is None or not sr.red:
                        if sl is not None:
                            sl.red = False
                        xpr.red = True
                        root = _rotate_right(root, xpr)
                        xp = x.parent
                        xpr = None if xp is None else xp.right
                    if xpr is not None:
                        xpr.red = False if xp is None else xp.red
                        sr = xpr.right
                        if sr is not None:
                            sr.red = False
                    if xp is not None:
                        xp.red = False
                        root = _rotate_left(root, xp)
                    x = root
        else:
            if xpl is not None and xpl.red:
                xpl.red = False
                xp.red = True
                root = _rotate_right(root, xp)
                xp = x.parent
                xpl = None if xp is None else xp.left
            if xpl is None:
                x = xp
            else:
                sl, sr = xpl.left, xpl.right
                if (sl is None or not sl.red) and (sr is None or not sr.red):
                    xpl.red = True
                    x = xp
                else:
                    if sl is None or not sl.red:
                        if sr is not None:
                            sr.red = False
                        xpl.red = True
                        root = _rotate_left(root, xpl)
                        xp = x.parent
                        xpl = None if xp is None else xp.left
                    if xpl is not None:
                        xpl.red = False if xp is None else xp.red
                        sl = xpl.left
                        if sl is not None:
                            sl.red = False
                    if xp is not None:
                        xp.red = False
                        root = _rotate_right(root, xp)
                    x = root


class _TreeBin:
    """A treeified bin: ``order`` is the ``next`` chain (iteration order), ``nodes`` maps
    each key to its :class:`_TreeNode`."""

    __slots__ = ("order", "nodes")

    def __init__(self, order: list, hfun):
        # HashMap.TreeNode.treeify followed by moveRootToFront.
        self.order = order
        self.nodes = {}
        root = None
        for k in order:
            x = _TreeNode(k, i32(hfun(k)))
            self.nodes[k] = x
            if root is None:
                root = x
                continue
            p = root
            while True:
                d = _tn_dir(x.hash, k, p)
                xp = p
                p = p.left if d <= 0 else p.right
                if p is None:
                    x.parent = xp
                    if d <= 0:
                        xp.left = x
                    else:
                        xp.right = x
                    root = _balance_insertion(root, x)
                    break
        self._move_root_to_front(root)

    def _root(self):
        r = self.nodes[self.order[0]]
        while r.parent is not None:
            r = r.parent
        return r

    def _move_root_to_front(self, root) -> None:
        if root is not None and self.order[0] != root.key:
            self.order.remove(root.key)
            self.order.insert(0, root.key)

    def put(self, k: int, h: int) -> None:
        """``putTreeVal`` for a key known to be absent: the new node goes into the
        ``next`` chain right after its tree parent."""
        h = i32(h)
        root = self._root()
        p = root
        while True:
            d = _tn_dir(h, k, p)
            xp = p
            p = p.left if d <= 0 else p.right
            if p is None:
                x = _TreeNode(k, h)
                self.nodes[k] = x
                if d <= 0:
                    xp.left = x
                else:
                    xp.right = x
                x.parent = xp
                self.order.insert(self.order.index(xp.key) + 1, k)
                self._move_root_to_front(_balance_insertion(root, x))
                return

    def remove(self, k: int) -> bool:
        """``removeTreeNode(map, tab, movable=true)``. Returns True when the bin must
        be untreeified (the caller then keeps only ``order``)."""
        root = self._root()  # root before the unlink (the chain head or its root())
        p = self.nodes.pop(k)
        self.order.remove(k)
        if not self.order:
            return True
        if root.right is None or root.left is None or root.left.left is None:
            return True
        pl, pr = p.left, p.right
        if pl is not None and pr is not None:
            s = pr
            while s.left is not None:
                s = s.left
            s.red, p.red = p.red, s.red
            sr = s.right
            pp = p.parent
            if s is pr:
                p.parent = s
                s.right = p
            else:
                sp = s.parent
                p.parent = sp
                if sp is not None:
                    if s is sp.left:
                        sp.left = p
                    else:
                        sp.right = p
                s.right = pr
                if pr is not None:
                    pr.parent = s
            p.left = None
            p.right = sr
            if sr is not None:
                sr.parent = p
            s.left = pl
            if pl is not None:
                pl.parent = s
            s.parent = pp
            if pp is None:
                root = s
            elif p is pp.left:
                pp.left = s
            else:
                pp.right = s
            replacement = sr if sr is not None else p
        elif pl is not None:
            replacement = pl
        elif pr is not None:
            replacement = pr
        else:
            replacement = p
        if replacement is not p:
            pp = replacement.parent = p.parent
            if pp is None:
                root = replacement
                replacement.red = False
            elif p is pp.left:
                pp.left = replacement
            else:
                pp.right = replacement
            p.left = p.right = p.parent = None
        r = root if p.red else _balance_deletion(root, replacement)
        if replacement is p:
            pp = p.parent
            p.parent = None
            if pp is not None:
                if p is pp.left:
                    pp.left = None
                elif p is pp.right:
                    pp.right = None
        self._move_root_to_front(r)
        return False


class JavaIntHashSet:
    """Iteration-order emulation of ``java.util.HashSet<Integer>`` (JDK 8+ HashMap).

    ``Integer.hashCode()`` is the value; the hash is spread as ``h ^ (h >>> 16)``; buckets
    are ``hash & (cap - 1)``; new keys go at the tail of their bin; a resize splits bins
    and keeps relative order; capacity never shrinks. A bin reaching 9 nodes while
    ``cap < 64`` forces a resize (``treeifyBin``); at ``cap >= 64`` it is treeified.
    Treeified bins are emulated exactly (``HashMap.TreeNode``): the red-black tree is
    kept, a new key is linked into the bin's chain right after its tree parent, the tree
    root is moved to the front of the chain, a resize splits a tree bin and
    re-treeifies or untreeifies (<= 6 nodes) each half, and ``remove`` follows
    ``removeTreeNode``. Verified against JDK 17 on random add/remove sequences.
    """

    __slots__ = ("_cap", "_tab", "_size", "_thr", "_trees")

    def __init__(self, items=()):
        self._cap = 0
        self._tab = None
        self._size = 0
        self._thr = 0
        self._trees = {}
        for k in items:
            self.add(k)

    @staticmethod
    def _h(k: int) -> int:
        h = k & 0xFFFFFFFF
        return h ^ (h >> 16)

    def _resize(self) -> None:
        ncap = 16 if self._cap == 0 else self._cap * 2
        ntab = [[] for _ in range(ncap)]
        ntrees = {}
        if self._tab is not None:
            mask = ncap - 1
            bit = self._cap
            for j, b in enumerate(self._tab):
                tree = self._trees.get(j)
                if tree is None:
                    for k in b:
                        ntab[self._h(k) & mask].append(k)
                    continue
                # TreeNode.split
                lo = [k for k in b if not self._h(k) & bit]
                hi = [k for k in b if self._h(k) & bit]
                for idx, half, other in ((j, lo, hi), (j + bit, hi, lo)):
                    if not half:
                        continue
                    if len(half) <= 6:          # untreeify
                        ntab[idx] = half
                    elif not other:             # whole tree moves unchanged
                        ntab[idx] = tree.order
                        ntrees[idx] = tree
                    else:
                        ntrees[idx] = _TreeBin(half, self._h)
                        ntab[idx] = half
        self._tab, self._cap, self._thr, self._trees = ntab, ncap, int(ncap * 0.75), ntrees

    def add(self, k: int) -> bool:
        if self._tab is None:
            self._resize()
        idx = self._h(k) & (self._cap - 1)
        b = self._tab[idx]
        if k in b:
            return False
        tree = self._trees.get(idx)
        if tree is not None:
            tree.put(k, self._h(k))
        else:
            b.append(k)
            if len(b) >= 9:
                if self._cap < 64:
                    self._resize()
                else:
                    self._trees[idx] = _TreeBin(b, self._h)
        self._size += 1
        if self._size > self._thr:
            self._resize()
        return True

    def remove(self, k: int) -> bool:
        if self._tab is None:
            return False
        idx = self._h(k) & (self._cap - 1)
        b = self._tab[idx]
        if k not in b:
            return False
        tree = self._trees.get(idx)
        if tree is None:
            b.remove(k)
        elif tree.remove(k):
            del self._trees[idx]
        self._size -= 1
        return True

    def __contains__(self, k: int) -> bool:
        if self._tab is None:
            return False
        return k in self._tab[self._h(k) & (self._cap - 1)]

    def __len__(self) -> int:
        return self._size

    def __iter__(self):
        if self._tab is None:
            return iter(())
        return (k for b in self._tab for k in b)

    def to_list(self) -> list[int]:
        return list(self)


def java_double_str(d: float) -> str:
    """``Double.toString(d)`` (JDK 17) for 1e-4 <= |d| < 1e16, plus 0, NaN and infinities.

    Uses the shortest round-trip digits (``repr``). JDK < 19 occasionally prints a longer
    digit string above 1e16; the original never prints such values.
    """
    if math.isnan(d):
        return "NaN"
    if math.isinf(d):
        return "Infinity" if d > 0 else "-Infinity"
    if d == 0.0:
        return "-0.0" if math.copysign(1.0, d) < 0 else "0.0"
    sign = "-" if d < 0 else ""
    a = abs(d)
    r = repr(a)
    if "e" in r:
        mant, exp = r.split("e")
        exp = int(exp)
    else:
        mant, exp = r, 0
    ip, fp = mant.split(".") if "." in mant else (mant, "")
    all_digits = ip + fp
    lead_zeros = len(all_digits) - len(all_digits.lstrip("0"))
    pt = len(ip) + exp - lead_zeros
    digits = all_digits.lstrip("0").rstrip("0") or "0"
    if 1e-3 <= a < 1e7:
        if pt <= 0:
            s = "0." + "0" * (-pt) + digits
        elif pt >= len(digits):
            s = digits + "0" * (pt - len(digits)) + ".0"
        else:
            s = digits[:pt] + "." + digits[pt:]
        return sign + s
    frac = digits[1:] or "0"
    return f"{sign}{digits[0]}.{frac}E{pt - 1}"
