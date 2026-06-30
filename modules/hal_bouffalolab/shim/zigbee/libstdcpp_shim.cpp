/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal libstdc++ shim for zigbee stack blobs.
 * Provides _Rb_tree, _List_node_base, and std::string functions
 * that the C++ blobs reference.
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

extern "C" void *pvPortMalloc(size_t size);
extern "C" void vPortFree(void *ptr);

namespace std {

/* Rb_tree node operations for std::set / std::map */
struct _Rb_tree_node_base {
	enum _Rb_tree_color { _S_red = false, _S_black = true };
	_Rb_tree_color _M_color;
	_Rb_tree_node_base *_M_parent;
	_Rb_tree_node_base *_M_left;
	_Rb_tree_node_base *_M_right;

	static _Rb_tree_node_base *_S_minimum(_Rb_tree_node_base *x)
	{
		while (x->_M_left != nullptr) {
			x = x->_M_left;
		}
		return x;
	}

	static _Rb_tree_node_base *_S_maximum(_Rb_tree_node_base *x)
	{
		while (x->_M_right != nullptr) {
			x = x->_M_right;
		}
		return x;
	}
};

_Rb_tree_node_base *_Rb_tree_increment(_Rb_tree_node_base *x)
{
	if (x->_M_right != nullptr) {
		x = x->_M_right;
		while (x->_M_left != nullptr) {
			x = x->_M_left;
		}
	} else {
		_Rb_tree_node_base *y = x->_M_parent;
		while (x == y->_M_right) {
			x = y;
			y = y->_M_parent;
		}
		if (x->_M_right != y) {
			x = y;
		}
	}
	return x;
}

const _Rb_tree_node_base *_Rb_tree_increment(const _Rb_tree_node_base *x)
{
	return _Rb_tree_increment(const_cast<_Rb_tree_node_base *>(x));
}

_Rb_tree_node_base *_Rb_tree_decrement(_Rb_tree_node_base *x)
{
	if (x->_M_color == _Rb_tree_node_base::_S_red && x->_M_parent->_M_parent == x) {
		x = x->_M_right;
	} else if (x->_M_left != nullptr) {
		_Rb_tree_node_base *y = x->_M_left;
		while (y->_M_right != nullptr) {
			y = y->_M_right;
		}
		x = y;
	} else {
		_Rb_tree_node_base *y = x->_M_parent;
		while (x == y->_M_left) {
			x = y;
			y = y->_M_parent;
		}
		x = y;
	}
	return x;
}

const _Rb_tree_node_base *_Rb_tree_decrement(const _Rb_tree_node_base *x)
{
	return _Rb_tree_decrement(const_cast<_Rb_tree_node_base *>(x));
}

static void _Rb_tree_rotate_left(_Rb_tree_node_base *x, _Rb_tree_node_base *&root)
{
	_Rb_tree_node_base *y = x->_M_right;

	x->_M_right = y->_M_left;
	if (y->_M_left != nullptr) {
		y->_M_left->_M_parent = x;
	}
	y->_M_parent = x->_M_parent;

	if (x == root) {
		root = y;
	} else if (x == x->_M_parent->_M_left) {
		x->_M_parent->_M_left = y;
	} else {
		x->_M_parent->_M_right = y;
	}
	y->_M_left = x;
	x->_M_parent = y;
}

static void _Rb_tree_rotate_right(_Rb_tree_node_base *x, _Rb_tree_node_base *&root)
{
	_Rb_tree_node_base *y = x->_M_left;

	x->_M_left = y->_M_right;
	if (y->_M_right != nullptr) {
		y->_M_right->_M_parent = x;
	}
	y->_M_parent = x->_M_parent;

	if (x == root) {
		root = y;
	} else if (x == x->_M_parent->_M_right) {
		x->_M_parent->_M_right = y;
	} else {
		x->_M_parent->_M_left = y;
	}
	y->_M_right = x;
	x->_M_parent = y;
}

void _Rb_tree_insert_and_rebalance(bool insert_left, _Rb_tree_node_base *x,
				   _Rb_tree_node_base *p, _Rb_tree_node_base &header)
{
	_Rb_tree_node_base *&root = header._M_parent;

	x->_M_parent = p;
	x->_M_left = nullptr;
	x->_M_right = nullptr;
	x->_M_color = _Rb_tree_node_base::_S_red;

	if (insert_left) {
		p->_M_left = x;
		if (p == &header) {
			header._M_parent = x;
			header._M_right = x;
		} else if (p == header._M_left) {
			header._M_left = x;
		}
	} else {
		p->_M_right = x;
		if (p == header._M_right) {
			header._M_right = x;
		}
	}

	while (x != root && x->_M_parent->_M_color == _Rb_tree_node_base::_S_red) {
		_Rb_tree_node_base *xpp = x->_M_parent->_M_parent;

		if (x->_M_parent == xpp->_M_left) {
			_Rb_tree_node_base *y = xpp->_M_right;
			if (y && y->_M_color == _Rb_tree_node_base::_S_red) {
				x->_M_parent->_M_color = _Rb_tree_node_base::_S_black;
				y->_M_color = _Rb_tree_node_base::_S_black;
				xpp->_M_color = _Rb_tree_node_base::_S_red;
				x = xpp;
			} else {
				if (x == x->_M_parent->_M_right) {
					x = x->_M_parent;
					_Rb_tree_rotate_left(x, root);
				}
				x->_M_parent->_M_color = _Rb_tree_node_base::_S_black;
				xpp->_M_color = _Rb_tree_node_base::_S_red;
				_Rb_tree_rotate_right(xpp, root);
			}
		} else {
			_Rb_tree_node_base *y = xpp->_M_left;
			if (y && y->_M_color == _Rb_tree_node_base::_S_red) {
				x->_M_parent->_M_color = _Rb_tree_node_base::_S_black;
				y->_M_color = _Rb_tree_node_base::_S_black;
				xpp->_M_color = _Rb_tree_node_base::_S_red;
				x = xpp;
			} else {
				if (x == x->_M_parent->_M_left) {
					x = x->_M_parent;
					_Rb_tree_rotate_right(x, root);
				}
				x->_M_parent->_M_color = _Rb_tree_node_base::_S_black;
				xpp->_M_color = _Rb_tree_node_base::_S_red;
				_Rb_tree_rotate_left(xpp, root);
			}
		}
	}
	root->_M_color = _Rb_tree_node_base::_S_black;
}

_Rb_tree_node_base *_Rb_tree_rebalance_for_erase(_Rb_tree_node_base *z,
						  _Rb_tree_node_base &header)
{
	_Rb_tree_node_base *&root = header._M_parent;
	_Rb_tree_node_base *&leftmost = header._M_left;
	_Rb_tree_node_base *&rightmost = header._M_right;
	_Rb_tree_node_base *y = z;
	_Rb_tree_node_base *x = nullptr;
	_Rb_tree_node_base *x_parent = nullptr;

	if (y->_M_left == nullptr) {
		x = y->_M_right;
	} else if (y->_M_right == nullptr) {
		x = y->_M_left;
	} else {
		y = y->_M_right;
		while (y->_M_left != nullptr) {
			y = y->_M_left;
		}
		x = y->_M_right;
	}

	if (y != z) {
		z->_M_left->_M_parent = y;
		y->_M_left = z->_M_left;
		if (y != z->_M_right) {
			x_parent = y->_M_parent;
			if (x) {
				x->_M_parent = y->_M_parent;
			}
			y->_M_parent->_M_left = x;
			y->_M_right = z->_M_right;
			z->_M_right->_M_parent = y;
		} else {
			x_parent = y;
		}
		if (root == z) {
			root = y;
		} else if (z->_M_parent->_M_left == z) {
			z->_M_parent->_M_left = y;
		} else {
			z->_M_parent->_M_right = y;
		}
		y->_M_parent = z->_M_parent;
		auto tmp = y->_M_color;
		y->_M_color = z->_M_color;
		z->_M_color = tmp;
		y = z;
	} else {
		x_parent = y->_M_parent;
		if (x) {
			x->_M_parent = y->_M_parent;
		}
		if (root == z) {
			root = x;
		} else if (z->_M_parent->_M_left == z) {
			z->_M_parent->_M_left = x;
		} else {
			z->_M_parent->_M_right = x;
		}
		if (leftmost == z) {
			if (z->_M_right == nullptr) {
				leftmost = z->_M_parent;
			} else {
				leftmost = _Rb_tree_node_base::_S_minimum(x);
			}
		}
		if (rightmost == z) {
			if (z->_M_left == nullptr) {
				rightmost = z->_M_parent;
			} else {
				rightmost = _Rb_tree_node_base::_S_maximum(x);
			}
		}
	}

	if (y->_M_color != _Rb_tree_node_base::_S_red) {
		while (x != root && (x == nullptr || x->_M_color == _Rb_tree_node_base::_S_black)) {
			if (x == x_parent->_M_left) {
				_Rb_tree_node_base *w = x_parent->_M_right;
				if (w->_M_color == _Rb_tree_node_base::_S_red) {
					w->_M_color = _Rb_tree_node_base::_S_black;
					x_parent->_M_color = _Rb_tree_node_base::_S_red;
					_Rb_tree_rotate_left(x_parent, root);
					w = x_parent->_M_right;
				}
				if ((w->_M_left == nullptr || w->_M_left->_M_color == _Rb_tree_node_base::_S_black) &&
				    (w->_M_right == nullptr || w->_M_right->_M_color == _Rb_tree_node_base::_S_black)) {
					w->_M_color = _Rb_tree_node_base::_S_red;
					x = x_parent;
					x_parent = x_parent->_M_parent;
				} else {
					if (w->_M_right == nullptr || w->_M_right->_M_color == _Rb_tree_node_base::_S_black) {
						w->_M_left->_M_color = _Rb_tree_node_base::_S_black;
						w->_M_color = _Rb_tree_node_base::_S_red;
						_Rb_tree_rotate_right(w, root);
						w = x_parent->_M_right;
					}
					w->_M_color = x_parent->_M_color;
					x_parent->_M_color = _Rb_tree_node_base::_S_black;
					if (w->_M_right) {
						w->_M_right->_M_color = _Rb_tree_node_base::_S_black;
					}
					_Rb_tree_rotate_left(x_parent, root);
					break;
				}
			} else {
				_Rb_tree_node_base *w = x_parent->_M_left;
				if (w->_M_color == _Rb_tree_node_base::_S_red) {
					w->_M_color = _Rb_tree_node_base::_S_black;
					x_parent->_M_color = _Rb_tree_node_base::_S_red;
					_Rb_tree_rotate_right(x_parent, root);
					w = x_parent->_M_left;
				}
				if ((w->_M_right == nullptr || w->_M_right->_M_color == _Rb_tree_node_base::_S_black) &&
				    (w->_M_left == nullptr || w->_M_left->_M_color == _Rb_tree_node_base::_S_black)) {
					w->_M_color = _Rb_tree_node_base::_S_red;
					x = x_parent;
					x_parent = x_parent->_M_parent;
				} else {
					if (w->_M_left == nullptr || w->_M_left->_M_color == _Rb_tree_node_base::_S_black) {
						w->_M_right->_M_color = _Rb_tree_node_base::_S_black;
						w->_M_color = _Rb_tree_node_base::_S_red;
						_Rb_tree_rotate_left(w, root);
						w = x_parent->_M_left;
					}
					w->_M_color = x_parent->_M_color;
					x_parent->_M_color = _Rb_tree_node_base::_S_black;
					if (w->_M_left) {
						w->_M_left->_M_color = _Rb_tree_node_base::_S_black;
					}
					_Rb_tree_rotate_right(x_parent, root);
					break;
				}
			}
		}
		if (x) {
			x->_M_color = _Rb_tree_node_base::_S_black;
		}
	}
	return y;
}

namespace __detail {

struct _List_node_base {
	_List_node_base *_M_next;
	_List_node_base *_M_prev;

	void _M_hook(_List_node_base *position);
	void _M_unhook();
	void _M_transfer(_List_node_base *first, _List_node_base *last);
	static void swap(_List_node_base &x, _List_node_base &y);
};

void _List_node_base::_M_hook(_List_node_base *position)
{
	_M_next = position;
	_M_prev = position->_M_prev;
	position->_M_prev->_M_next = this;
	position->_M_prev = this;
}

void _List_node_base::_M_unhook()
{
	_M_prev->_M_next = _M_next;
	_M_next->_M_prev = _M_prev;
}

void _List_node_base::_M_transfer(_List_node_base *first, _List_node_base *last)
{
	if (this != last) {
		last->_M_prev->_M_next = this;
		first->_M_prev->_M_next = last;
		this->_M_prev->_M_next = first;

		_List_node_base *tmp = this->_M_prev;
		this->_M_prev = last->_M_prev;
		last->_M_prev = first->_M_prev;
		first->_M_prev = tmp;
	}
}

void _List_node_base::swap(_List_node_base &x, _List_node_base &y)
{
	if (x._M_next != &x) {
		if (y._M_next != &y) {
			_List_node_base *tmp_next = x._M_next;
			_List_node_base *tmp_prev = x._M_prev;
			x._M_next = y._M_next;
			x._M_prev = y._M_prev;
			y._M_next = tmp_next;
			y._M_prev = tmp_prev;
			x._M_next->_M_prev = &x;
			x._M_prev->_M_next = &x;
			y._M_next->_M_prev = &y;
			y._M_prev->_M_next = &y;
		} else {
			y._M_next = x._M_next;
			y._M_prev = x._M_prev;
			y._M_next->_M_prev = &y;
			y._M_prev->_M_next = &y;
			x._M_next = &x;
			x._M_prev = &x;
		}
	} else if (y._M_next != &y) {
		x._M_next = y._M_next;
		x._M_prev = y._M_prev;
		x._M_next->_M_prev = &x;
		x._M_prev->_M_next = &x;
		y._M_next = &y;
		y._M_prev = &y;
	}
}

} /* namespace __detail */

namespace __cxx11 {

class basic_string_base {
public:
	struct _Alloc_hider {
		char *_M_p;
	};

	_Alloc_hider _M_dataplus;
	size_t _M_string_length;

	enum { _S_local_capacity = 15 };
	union {
		char _M_local_buf[_S_local_capacity + 1];
		size_t _M_allocated_capacity;
	};

	char *_M_local_data()
	{
		return _M_local_buf;
	}

	bool _M_is_local() const
	{
		return _M_dataplus._M_p == const_cast<basic_string_base *>(this)->_M_local_buf;
	}
};

} /* namespace __cxx11 */

} /* namespace std */

/* std::__cxx11::basic_string methods referenced by blobs */
extern "C" {

void _ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE10_M_disposeEv(void *_this)
{
	auto *s = reinterpret_cast<std::__cxx11::basic_string_base *>(_this);

	if (!s->_M_is_local()) {
		vPortFree(s->_M_dataplus._M_p);
	}
	s->_M_dataplus._M_p = s->_M_local_data();
	s->_M_string_length = 0;
	s->_M_local_buf[0] = '\0';
}

void _ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE13_S_copy_charsEPcPKcS7_(
	char *dst, const char *src, const char *end)
{
	memcpy(dst, src, (size_t)(end - src));
}

void _ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE14_M_replace_auxEjjjc(
	void *_this, size_t pos, size_t len1, size_t len2, char c)
{
	(void)_this;
	(void)pos;
	(void)len1;
	(void)len2;
	(void)c;
}

void _ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE7reserveEj(void *_this, size_t n)
{
	(void)_this;
	(void)n;
}

void _ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE9_M_appendEPKcj(
	void *_this, const char *s, size_t n)
{
	(void)_this;
	(void)s;
	(void)n;
}

char *_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE9_M_createERjj(
	void *_this, size_t *capacity, size_t old_capacity)
{
	(void)_this;
	(void)old_capacity;
	char *p = (char *)pvPortMalloc(*capacity + 1);

	if (p == nullptr) {
		*capacity = 0;
	}
	return p;
}

void _ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE9push_backEc(void *_this, char c)
{
	(void)_this;
	(void)c;
}

void _ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEC1EOS4_(void *_this, void *other)
{
	memcpy(_this, other, sizeof(std::__cxx11::basic_string_base));
	auto *src = reinterpret_cast<std::__cxx11::basic_string_base *>(other);
	auto *dst = reinterpret_cast<std::__cxx11::basic_string_base *>(_this);

	if (src->_M_is_local()) {
		dst->_M_dataplus._M_p = dst->_M_local_data();
	}
	src->_M_dataplus._M_p = src->_M_local_data();
	src->_M_string_length = 0;
	src->_M_local_buf[0] = '\0';
}

} /* extern "C" */
