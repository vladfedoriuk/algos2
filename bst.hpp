#ifndef _BST_H
#define _BST_H

#include<iostream>
#include<functional>
#include<exception>
#include<stack>

template<typename T>
class BST {
private:
	std::size_t current_size;
	std::size_t max_depth;
	struct Node {
		T data;
		Node* right;
		Node* left;
		Node* up;
		std::size_t depth;
		Node();
		Node(const T& d);
		Node(const T& d, Node* l, Node* r, Node* u);
		Node(const T& d, Node* u);
	};
	void maximize_depth(Node* u);
	static void copy_tree(BST<T>& bst, Node* original, Node* duplicate, Node* upper);
public:
	Node* root;
	Node* most_right() const{
		Node* p = root;
		if (p) while (p->right) p = p->right;
		return p;
	};
	Node* most_left() const{
		Node* p = root;
		if (p) while (p->left) p = p->left;
		return p;
	};
	Node* most_left(Node* k) const{
		Node* p = k;
		if (p) while (p->left) p = p->left;
		return p;
	};
	void clear(){
        std::function<void(Node*)>  f = [](Node* p){ delete p; };
        traverse_post_order(root, f);
	};
	using node_ptr = Node*;
	void traverse_post_order(Node* p, const std::function<void(Node*)> & f);
	void traverse_in_order(Node* p, const std::function<void(Node*)> & f);
	void traverse_in_order(std::function<void(const T&)> f) const;
	void traverse_pre_order(Node* p, const std::function<void(Node*)> & f);
	void traverse_iterative(const std::function<void(Node*)> & f);
	void set_size(const std::size_t& siz) {
		current_size = siz;
	};
	void set_depth(const std::size_t& depth) {
		max_depth = depth;
	};
   
	BST();
	~BST();
	BST(const BST<T>& tree);
	BST(BST<T>&& tree);
	BST<T>& operator=(const BST<T>& tree);
	BST<T>& operator=(BST<T>&& tree);
	bool insert(const T& data);
	std::size_t size() const;
	const T& minimum() const;
	const T& maximum() const;
	std::size_t depth() const;
	void printInorder();
	void printPreorder();
	void printPostorder();
	node_ptr search(const T& data) const
	{
		Node* p = root;
		while (p && p->data != data)
			p = (data < p->data) ? p->left : p->right;
		return p;
	};

	static node_ptr find_succ(node_ptr p) {
		if (!p) throw std::runtime_error("nullptr as a find_succ() argument");
		Node* succ = nullptr;
		if (p->right) {
			succ = p->right;
			while (succ->left) succ = succ->left;
		}
		else {
			Node* r = p;
			succ = p->up;
			while (succ && r == succ->right) {
				r = succ;
				succ = succ->up;
			}
		}
		return succ;
	};

	void delete_node(Node* p);
	struct iterator;

	iterator begin() const {
		return iterator(most_left());
	};
	iterator end() const{
		return iterator(nullptr);
	};
};

template<typename T>
struct BST<T>::iterator{
	Node* ptr;
	iterator(const iterator& it){
	    ptr = it.ptr;
	};
	iterator() : ptr(nullptr) {

	};
    iterator(const BST<T>& bst) : ptr(bst.most_left()) {

	};
	iterator(Node* ptr) : ptr(ptr) {

	};
	iterator& operator++() {
		if (ptr!=nullptr) {
			ptr = find_succ(ptr);
		}
		return *this;
	};
	iterator operator++(int){
		Node* new_ptr;
		if (ptr!=nullptr) {
			new_ptr = find_succ(ptr);
		}
		else {
			new_ptr = ptr;
		}
        iterator res(ptr);
		ptr = new_ptr;
		return res;
	};

	iterator& operator=(const iterator& it) {
		this->ptr = it.ptr;
		return *this;
	};

	bool operator==(const iterator& a) const {
		return this->ptr == a.ptr;
	};

	bool operator!=(const iterator& a) const {
		return this->ptr != a.ptr;
	};

	const T& operator*() const{
		return ptr->data;
	};
};


template<typename T>
inline void BST<T>::copy_tree(BST<T>& bst, Node * original, Node * duplicate, Node * upper)
{
	if (original!=nullptr)
	{
	    duplicate = new Node();
		duplicate->data = original->data;
		duplicate->up = upper;
		duplicate->depth = original->depth;
		if (!upper) {
			bst.root = duplicate;
		}
		else {
			if (original->up->left == original) duplicate->up->left = duplicate;
			if (original->up->right == original) duplicate->up->right = duplicate;
		}
		copy_tree(bst, original->left, duplicate->left, duplicate);
		copy_tree(bst, original->right, duplicate->right, duplicate);
	}
	else {
		duplicate = nullptr;
	}
};

template<typename T>
void BST<T>::traverse_post_order(Node* p, const std::function<void(Node*)>& f) {
	if (p!=nullptr) {
		traverse_post_order(p->left, f);
		traverse_post_order(p->right, f);
		f(p);
	}
};

template<typename T>
inline void BST<T>::traverse_in_order(Node * p, const std::function<void(Node*)>& f)
{
	if (p!=nullptr) {
		traverse_in_order(p->left, f);
		f(p);
		traverse_in_order(p->right, f);
	}
}
template<typename T>
inline void BST<T>::traverse_in_order(std::function<void(const T&)> f) const
{
	Node* p = root;
	std::stack<Node*> st;
	while (p || !st.empty()) {
		if (p) {
			st.push(p);
			p = p->left;
		}
		else {
			p = st.top();
			st.pop();
			f(p->data);
			p = p->right;
		}
	}
};

template<typename T>
inline void BST<T>::traverse_pre_order(Node * p, const std::function<void(Node*)>& f)
{
	if (p!=nullptr) {
		f(p);
		traverse_pre_order(p->left, f);
		traverse_pre_order(p->right, f);
	}
};

template<typename T>
inline void BST<T>::traverse_iterative(const std::function<void(Node*)>& f)
{
	Node* p = root;
	std::stack<Node*> st;
	while (p || !st.empty()) {
		if (p) {
			st.push(p);
			p = p->left;
		}
		else {
			p = st.top();
			st.pop();
			f(p);
			p = p->right;
		}
	}
};

template<typename T>
inline void BST<T>::maximize_depth(Node* u)
{
	if (u->depth > max_depth) {
		max_depth = u->depth;
	}
};

template<typename T>
inline void BST<T>::printInorder()
{
	std::function<void(Node*)> f = [](Node* x) { std::cout << x->data << " "; };
	//traverse_in_order(root, f);
	traverse_iterative(f);
	std::cout << std::endl;
};

template<typename T>
inline void BST<T>::printPreorder()
{
	std::function<void(Node*)> f = [](Node* x) { std::cout << x->data << " "; };
	traverse_pre_order(root, f);
	std::cout << std::endl;

};

template<typename T>
inline void BST<T>::printPostorder()
{
	std::function<void(Node*)> f = [](Node* x) { std::cout << x->data << " "; };
	traverse_post_order(root, f);
	std::cout << std::endl;
}
template<typename T>
inline void BST<T>::delete_node(Node * p)
{
	if (!p) throw std::runtime_error("wrong argument has been passed to delete_node()");
	if (!p->left && !p->right) {
		Node* r = p;
		if(p->up && p == p->up->left) p->up->left  = nullptr;
		if(p->up && p == p->up->right) p->up->right = nullptr;
		delete r;
		current_size--;
	}
	else if (p->left && !p->right) {
			Node* r = p;
			if (p->up) {
				p->up->left = p->left;
			}
			p->left->up = p->up;
			delete r;
			current_size--;
	}
	else if (p->right && !p->left) {
			Node* r = p;
			if (p->up) {
				p->up->right = p->right;
			}
			p->right->up = p->up;
			delete r;
			current_size--;
	}
	else {
		Node* succ = find_succ(p);
		if (succ) {
			p->data = succ->data;
			delete_node(succ);
		}
	}
};

template<typename T>
BST<T>::BST() : current_size(0), max_depth(0), root(nullptr){

};

template<typename T>
inline BST<T>::~BST()
{
	std::function<void(Node*)> f = [](Node* p) { delete p; };
	traverse_post_order(root, f);
};
template<typename T>
inline BST<T>::BST(const BST & tree): current_size(tree.size()), max_depth(tree.depth()), root(nullptr)
{
	//std::cout << "cc-tor" << std::endl;
	copy_tree(*this, tree.root, root, nullptr);

};

template<typename T>
inline BST<T>::BST(BST<T> && tree): current_size(std::move(tree.size())), max_depth(std::move(tree.depth())), root(nullptr){
	//std::cout << "mc-tor" << std::endl;
	/*copy_tree(*this, tree.root, root, nullptr);
	tree.traverse_post_order(tree.root, f);
	tree.root = nullptr;*/
	root = std::move(tree.root);
	tree.root = nullptr;
	tree.set_depth(0);
	tree.set_size(0);
};

template<typename T>
inline BST<T>& BST<T>::operator=(const BST<T>& tree)
{
	//std::cout << "ca op=" << std::endl;
	std::function<void(Node*)> f = [](Node* p) { delete p; p = nullptr; };
	traverse_post_order(root, f);
	root = nullptr;
	copy_tree(*this, tree.root, root, nullptr);
	current_size = tree.size();
	max_depth = tree.depth();
	return *this;
}
template<typename T>
inline BST<T>& BST<T>::operator=(BST<T>&& tree)
{
	//std::cout << "ma op =" << std::endl;
	std::function<void(Node*)> f = [](Node* p) {delete p; p = nullptr; };
	traverse_post_order(root, f);
	root = nullptr;
	current_size = std::move(tree.size());
	max_depth = std::move(tree.depth());
	/*copy_tree(*this, tree.root, root, nullptr);
	tree.traverse_post_order(tree.root, f);*/
	root = std::move(tree.root);
	tree.set_size(0);
	tree.set_depth(0);
	tree.root = nullptr;
	return *this;
};

template<typename T>
inline std::size_t BST<T>::size() const
{
	return current_size;
};

template<typename T>
inline const T& BST<T>::minimum() const
{
	Node* p = root;
	if (p) while (p->left) p = p->left;
	if (p) return p->data; else throw std::underflow_error("the tree is empty");
};

template<typename T>
inline const T& BST<T>::maximum() const
{
	Node* p = root;
	if (p) while (p->right) p = p->right;
	if (p) return p->data; else throw std::underflow_error("the tree is empty");
};

template<typename T>
inline std::size_t BST<T>::depth() const
{
	return max_depth;
};

template<typename T>
inline BST<T>::Node::Node():data(T()), right(nullptr), left(nullptr), up(nullptr), depth(0)
{
};

template<typename T>
BST<T>::Node::Node(const T& d): data(d), left(nullptr), right(nullptr), up(nullptr), depth(0){

};

template<typename T>
BST<T>::Node::Node(const T& d, Node* l, Node* r, Node* u): data(d), left(l), right(r), up(u){
	if (u) {
		this->depth = u->depth + 1;
	}
	else this->depth = 0;
};

template<typename T>
inline BST<T>::Node::Node(const T& d, Node * u) : data(d), up(u), left(nullptr), right(nullptr){
	if (u) {
		this->depth = u->depth + 1;
	}
	else this->depth = 0;
};

template<typename T>
bool BST<T>::insert(const T& data) {
	Node* p = root;
	if (!p) {
		root = new Node(data);
		current_size++;
	}
	else {
		while (true) {
			if (data < p->data) {
				if (!p->left) {
					p->left = new Node(data, p);
					maximize_depth(p->left);
					current_size++;
					break;
				}
				else {
					p = p->left;
				}
			}
			else if (data > p->data) {
				if (!p->right) {
					p->right = new Node(data, p);
					maximize_depth(p->right);
					current_size++;
					break;
				}
				else {
					p = p->right;
				}
			} else if(data == p->data){
				return false;
			}
		}
	}
	return true;
}



#endif