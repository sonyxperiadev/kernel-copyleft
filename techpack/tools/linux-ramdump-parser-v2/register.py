# Copyright (c) 2013-2014, 2020 The Linux Foundation. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

import bitops


class Register(object):

    """Represents a register (or any general field partitioning of
    bits). Provides easy read and write access to the fields in the
    register by taking care of all bit-shifting automatically. Fields
    can be defined at __init__ time using kwargs or can be added
    dynamically with ``add_field``. Fields are accessible as instance
    attributes.

    For example:

    >>> abc = Register(0x42, stuff=(2, 0))
    >>> abc
    value: 0x42 {stuff[2:0]=>0x2}
    >>> hex(abc.value)
    '0x42'
    >>> hex(abc.stuff)
    '0x2'
    >>> abc.stuff = 1
    >>> hex(abc.value)
    '0x41'
    >>> abc.add_field("other", (8, 4))
    >>> hex(abc.other)
    '0x4'
    >>> abc.other = 0x3
    >>> hex(abc.value)
    '0x31'
    >>> abc.add_field("just_a_bit", (4, 4))
    >>> abc.just_a_bit
    1
    >>> abc.just_a_bit = 0
    >>> abc.just_a_bit
    0
    >>> hex(abc.value)
    '0x21'
    >>> abc.value = 0
    >>> abc.value
    0
    >>> abc.value = 42
    >>> abc.value
    42
    >>> abc.zero()
    >>> abc.value
    0

    You can also overlay fields on top of each other without problems:

    >>> abc.add_field("another_other", (8, 0))
    >>> abc.another_other = 0x5
    >>> hex(abc.value)
    '0x5'

    We also handle ``None`` values:

    >>> r = Register(None, h=(3,0))
    >>> r
    value: None
    >>> r.h
    >>> r.h = 3
    >>> r
    value: 0x3 {h[3:0]=>0x3}

    """

    def __init__(self, value=0, **kwargs):
        """Register constructor.

        kwargs should represent the fields in this object. Their
        values should be 2-tuples of the form (msb, lsb).

        """

        # All the object.__setattr__ stuff is to prevent us from going
        # into our __setattr__ method here (which would try to access
        # these again and would then recurse inifitely)
        object.__setattr__(self, 'value', value)
        object.__setattr__(self, '_regs', {})
        for (k, v) in kwargs.items():
            self.add_field(k, v)

    def add_field(self, field, bitrange):
        """Add field to Register.

        bitrange should be the same format as the kwargs in __init__
        (i.e. (msb, lsb)).

        """
        self._regs[field] = bitrange

    def zero(self):
        object.__setattr__(self, 'value', 0)

    def __dir__(self):
        return self.__dict__.keys() + self._regs.keys()

    def __getattr__(self, name):
        if name not in self._regs:
            raise AttributeError
        if self.value is None:
            return None
        msb, lsb = self._regs[name]
        return bitops.bvalsel(msb, lsb, self.value)

    def __setattr__(self, name, newvalue):
        if name == 'value':
            object.__setattr__(self, 'value', newvalue)
            return
        if name not in self._regs:
            raise AttributeError
        if self.value is None:
            object.__setattr__(self, 'value', 0)
        msb, lsb = self._regs[name]
        val = self.value & (~bitops.bm(msb, lsb))
        val |= newvalue << lsb
        # can't access self.value directly since that would cause
        # infinite recursion to __setattr__
        object.__setattr__(self, 'value', val)

    def __eq__(self, other):
        """Two Register objects are defined to be equal if they have the same
        fields and all of those fields have the same values.

        >>> r1 = Register(0xf, top=(7, 4), bottom=(3, 0))
        >>> r2 = Register(0, top=(7, 4), bottom=(3, 0))
        >>> r1 == r2
        False
        >>> r2.bottom = 0xf
        >>> r1 == r2
        True
        >>> r2.top = 0xf
        >>> r1 == r2
        False

        """
        if self._regs != other._regs:
            return False
        for r in self._regs:
            if getattr(self, r) != getattr(other, r):
                return False
        return True

    def __repr__(self):
        if self.value is None:
            return 'value: None'
        ret = []
        for r in sorted(self._regs, key=self._regs.get, reverse=True):
            msb, lsb = self._regs[r]
            val = bitops.bvalsel(msb, lsb, self.value)
            ret.append('%s[%d:%d]=>0x%0x' % (r, msb, lsb, val))
        return 'value: 0x%x {%s}' % (self.value, ', '.join(ret))

if __name__ == "__main__":
    import doctest
    doctest.testmod()
