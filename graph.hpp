//
// Created by Vlad_F on 27.04.2020.
//

#ifndef ALGOS2_GRAPH_HPP
#define ALGOS2_GRAPH_HPP

#include <cstdint>
#include <utility>
#include <vector>
#include <stack>
#include <queue>
#include<vector>
#include <iostream>
#include<optional>
#include<exception>
#include<functional>
#include<algorithm>
#include<cmath>
#include"vertices_iterator.hpp"
#include"edges_iterator.hpp"

template <typename V, typename E>
class Graph
{
private:
    std::vector<std::vector<std::optional<E>>> matrix;
    std::vector<V> vertices;
    std::size_t edges;
public:
    using matrix_type = std::vector<std::vector<std::optional<E>>>;

    using ConstVerticesIterator = VIterator<true, V>;
    using VerticesIterator = VIterator<false, V>;

    using ConstEdgesIterator = EIterator<true, E>;
    using EdgesIterator = EIterator<false, E>;

    class DFSIterator
    {
        Graph<V, E>* g;
        std::stack<std::vector<std::pair<std::size_t, V>>> st;
        std::vector<bool> visited;
        std::size_t node_id;
        std::optional<V> node;
    public:
        Graph<V, E>* get_g() const;
        const std::stack<std::vector<std::pair<std::size_t, V>>>& get_stack() const;
        const std::vector<bool>& get_visited() const;
        const std::size_t& get_node_id() const;
        const std::optional<V>& get_node() const;
        DFSIterator(Graph<V, E>* g);
        DFSIterator(Graph<V, E>* g, std::size_t node_id);
        DFSIterator(const DFSIterator& dfsi);
        bool operator==(const DFSIterator& dfsi) const;
        bool operator!=(const DFSIterator& dfsi) const;
        DFSIterator& operator++();
        DFSIterator operator++(int);
        V& operator*() const;
        V* operator->() const;
        operator bool() const;
    };

    class BFSIterator
    {
        Graph<V, E>* g;
        std::queue<std::vector<std::pair<std::size_t, V>>> q;
        std::vector<bool> visited;
        std::size_t node_id;
        std::optional<V> node;
    public:
        Graph<V, E>* get_g() const;
        const std::queue<std::vector<std::pair<std::size_t, V>>>& get_queue() const;
        const std::vector<bool>& get_visited() const;
        const std::size_t& get_node_id() const;
        const std::optional<V>& get_node() const;
        BFSIterator(Graph<V, E>* g);
        BFSIterator(Graph<V, E>* g, std::size_t node_id);
        BFSIterator(const BFSIterator& dfsi);
        bool operator==(const BFSIterator& dfsi) const;
        bool operator!=(const BFSIterator& dfsi) const;
        BFSIterator& operator++();
        BFSIterator operator++(int);
        V& operator*() const;
        V* operator->() const;
        operator bool() const;
    };


// ...

public:
    Graph();
    Graph(const Graph<V, E> &source);
    Graph(Graph<V, E> &&source);
    Graph& operator=(const Graph<V, E> &source);
    Graph& operator=(Graph<V, E> &&source);
    ~Graph();

    const std::vector<std::vector<std::optional<E>>>& get_matrix() const;
    const std::vector<V>& get_vertices() const;

    VerticesIterator insertVertex(const V &vertex_data);
    std::pair<EdgesIterator, bool> insertEdge(std::size_t vertex1_id, std::size_t vertex2_id, const E &label = E(), bool replace = true);
    VerticesIterator removeVertex(std::size_t vertex_id);
    EdgesIterator removeEdge(std::size_t vertex1_id, std::size_t vertex2_id);
    bool edgeExist(std::size_t vertex1_id, std::size_t vertex2_id) const;
    std::size_t nrOfVertices() const;
    std::size_t nrOfEdges() const;
    void printNeighborhoodMatrix() const;
    VerticesIterator vertex(std::size_t vertex_id);
    ConstVerticesIterator vertex(std::size_t vertex_id) const;

    const V& vertexData(std::size_t vertex_id) const;
    V& vertexData(std::size_t vertex_id);
    EdgesIterator edge(std::size_t vertex1_id, std::size_t vertex2_id);
    ConstEdgesIterator edge(std::size_t vertex1_id, std::size_t vertex2_id) const;
    const E& edgeLabel(std::size_t vertex1_id, std::size_t vertex2_id) const;
    E& edgeLabel(std::size_t vertex1_id, std::size_t vertex2_id);
    ConstVerticesIterator begin() const{ return beginVertices(); }
    ConstVerticesIterator end() const{ return endVertices(); }
    ConstVerticesIterator beginVertices() const;
    VerticesIterator beginVertices();
    ConstVerticesIterator endVertices() const;
    VerticesIterator endVertices();
    ConstEdgesIterator beginEdges() const;
    EdgesIterator beginEdges();
    ConstEdgesIterator endEdges() const;
    EdgesIterator endEdges();
    DFSIterator beginDFS(std::size_t vertex_id);
    DFSIterator endDFS();
    BFSIterator beginBFS(std::size_t vertex_id);
    BFSIterator endBFS();
};

template<typename V, typename E>
Graph<V, E> *Graph<V, E>::BFSIterator::get_g() const {
    return g;
}

template<typename V, typename E>
const std::queue<std::vector<std::pair<std::size_t, V>>> &Graph<V, E>::BFSIterator::get_queue() const {
    return q;
}

template<typename V, typename E>
const std::vector<bool> &Graph<V, E>::BFSIterator::get_visited() const {
    return visited;
}

template<typename V, typename E>
const std::size_t &Graph<V, E>::BFSIterator::get_node_id() const {
    return node_id;
}

template<typename V, typename E>
const std::optional<V> &Graph<V, E>::BFSIterator::get_node() const {
    return node;
}

template<typename V, typename E>
Graph<V, E>::BFSIterator::BFSIterator(Graph<V, E> *g): g(g), node(std::optional<V>()) {

}

template<typename V, typename E>
Graph<V, E>::BFSIterator::BFSIterator(Graph<V, E> *g, std::size_t node_id): g(g), node_id(node_id),
visited(std::vector<bool>(g->nrOfVertices(), false)), node(std::optional<V>()) {

    auto it = typename Graph<V, E>::ConstEdgesIterator((&g->get_matrix()), node_id, 0);
    std::vector<std::pair<std::size_t, V>> neighbors;
    while (it && it.v1id() == node_id && it.v2id() < g->nrOfVertices()) {
        if (!visited[it.v2id()]) neighbors.push_back(std::make_pair(it.v2id(), g->vertexData(it.v2id())));
        ++it;
    }
    if(!neighbors.empty()) q.push(std::move(neighbors));
    node.emplace(g->vertexData(node_id));
    visited[node_id] = true;

}

template<typename V, typename E>
Graph<V, E>::BFSIterator::BFSIterator(const Graph::BFSIterator &dfsi): g(dfsi.get_g()), q(dfsi.get_queue()),
visited(dfsi.get_visited()), node_id(dfsi.get_node_id()), node(dfsi.get_node()) {

}

template<typename V, typename E>
bool Graph<V, E>::BFSIterator::operator==(const Graph::BFSIterator &dfsi) const {
    if (g!=dfsi.get_g()) return false;
    if(!dfsi.get_node().has_value() && !node.has_value()) return true;
    if (dfsi.get_node().has_value() && node.has_value()) return node.value() == dfsi.get_node().value();
    return false;
}

template<typename V, typename E>
bool Graph<V, E>::BFSIterator::operator!=(const Graph::BFSIterator &dfsi) const {
    return !(*this == dfsi);
}

template<typename V, typename E>
typename Graph<V, E>::BFSIterator &Graph<V, E>::BFSIterator::operator++() {
    if(!q.empty()) {

        while(!q.empty()) {

            std::size_t prev_node_id = node_id;
            auto &neighbors = q.front();
            for (const auto &e: neighbors) {
                if (!visited[e.first]) {
                    node_id = e.first;
                    node.emplace(e.second);
                    visited[e.first] = true;

                    auto it = typename Graph<V, E>::ConstEdgesIterator((&g->get_matrix()), e.first, 0);
                    std::vector<std::pair<std::size_t, V>> new_neighbors;
                    while (it && it.v1id() == e.first && it.v2id() < g->nrOfVertices()) {
                        if (!visited[it.v2id()])
                            new_neighbors.push_back(std::make_pair(it.v2id(), g->vertexData(it.v2id())));
                        ++it;
                    }

                    if (!new_neighbors.empty()) {
                        q.push(std::move(new_neighbors));
                    }
                    break;
                }
            }

            if (prev_node_id!=node_id) {
                break;
            } else {
                q.pop();
                if(q.empty()) node.reset();
            }
        }
    }else {
        node.reset();
    }
    return *this;
}

template<typename V, typename E>
typename Graph<V, E>::BFSIterator Graph<V, E>::BFSIterator::operator++(int) {
    auto it = Graph::BFSIterator(*this);
    *this++;
    return it;

}

template<typename V, typename E>
V &Graph<V, E>::BFSIterator::operator*() const {
    return g->vertexData(node_id);
}

template<typename V, typename E>
V *Graph<V, E>::BFSIterator::operator->() const {
    return &node.value();
}

template<typename V, typename E>
Graph<V, E>::BFSIterator::operator bool() const {
    return node.has_value();
}


template<typename V, typename E>
Graph<V, E>::DFSIterator::DFSIterator(Graph<V, E> *g, std::size_t node_id) : g(g), node_id(node_id),
visited(std::vector<bool>(g->nrOfVertices(), false)), node(std::optional<V>()) {

        auto it = typename Graph<V, E>::ConstEdgesIterator((&g->get_matrix()), node_id, 0);
        std::vector<std::pair<std::size_t, V>> neighbors;
        while (it && it.v1id() == node_id && it.v2id() < g->nrOfVertices()) {
            if (!visited[it.v2id()]) neighbors.push_back(std::make_pair(it.v2id(), g->vertexData(it.v2id())));
            ++it;
        }
        if(!neighbors.empty()) st.push(std::move(neighbors));
        node.emplace(g->vertexData(node_id));
        visited[node_id] = true;

}

template<typename V, typename E>
Graph<V, E>::DFSIterator::DFSIterator(const DFSIterator& dfsi): g(dfsi.get_g()), st(dfsi.get_stack()),
visited(dfsi.get_visited()), node_id(dfsi.get_node_id()), node(dfsi.get_node()){

}

template<typename V, typename E>
Graph<V, E> *Graph<V, E>::DFSIterator::get_g() const{
    return g;
}

template<typename V, typename E>
const std::stack<std::vector<std::pair<std::size_t, V>>>& Graph<V, E>::DFSIterator::get_stack() const{
    return st;
}

template<typename V, typename E>
const std::vector<bool>& Graph<V, E>::DFSIterator::get_visited() const{
    return visited;
}

template<typename V, typename E>
bool Graph<V, E>::DFSIterator::operator==(const Graph::DFSIterator &dfsi) const {
    if (g!=dfsi.get_g()) return false;
    if(!dfsi.get_node().has_value() && !node.has_value()) return true;
    if (dfsi.get_node().has_value() && node.has_value()) return node.value() == dfsi.get_node().value();
    return false;
}

template<typename V, typename E>
bool Graph<V, E>::DFSIterator::operator!=(const Graph::DFSIterator &dfsi) const {
    return !(*this==dfsi);
}

template<typename V, typename E>
typename Graph<V, E>::DFSIterator &Graph<V, E>::DFSIterator::operator++() {
    if(!st.empty()) {

        while(!st.empty()) {

            std::size_t size = st.size();
            auto &neighbors = st.top();

            for (const auto &e: neighbors) {
                if (!visited[e.first]) {

                    node_id = e.first;
                    node.emplace(e.second);
                    visited[e.first] = true;

                    auto it = typename Graph<V, E>::ConstEdgesIterator((&g->get_matrix()), e.first, 0);
                    std::vector<std::pair<std::size_t, V>> new_neighbors;
                    while (it && it.v1id() == e.first && it.v2id() < g->nrOfVertices()) {
                        if (!visited[it.v2id()])
                            new_neighbors.push_back(std::make_pair(it.v2id(), g->vertexData(it.v2id())));
                        ++it;
                    }

                    if (!new_neighbors.empty()){
                        st.push(std::move(new_neighbors));
                    }

                    break;
                }
            }
            if (size!=st.size()) {
               break;
            } else {
                st.pop();
            }

        }
    }else {
        node.reset();
    }
    return *this;
}

template<typename V, typename E>
V &Graph<V, E>::DFSIterator::operator*() const {
    return g->vertexData(node_id);
}

template<typename V, typename E>
V *Graph<V, E>::DFSIterator::operator->() const {
    return &g->vertexData(node_id);
}

template<typename V, typename E>
Graph<V, E>::DFSIterator::operator bool() const {
    return node.has_value();
}

template<typename V, typename E>
typename Graph<V, E>::DFSIterator Graph<V, E>::DFSIterator::operator++(int) {
    auto prev_iter = DFSIterator(*this);
    ++*this;
    return prev_iter;
}

template<typename V, typename E>
const std::size_t &Graph<V, E>::DFSIterator::get_node_id() const {
    return node_id;
}

template<typename V, typename E>
const std::optional<V> &Graph<V, E>::DFSIterator::get_node() const {
    return node;
}

template<typename V, typename E>
Graph<V, E>::DFSIterator::DFSIterator(Graph<V, E> *g): g(g), node(std::optional<V>()) {

}

template<typename V, typename E>
Graph<V, E>::Graph(): edges(0) {

}

template<typename V, typename E>
Graph<V, E>::Graph(const Graph<V, E> &source): matrix(source.get_matrix()), vertices(source.get_vertices()), edges(source.nrOfEdges()){

}

template<typename V, typename E>
const std::vector<std::vector<std::optional<E>>> &Graph<V, E>::get_matrix() const{
    return matrix;
}

template<typename V, typename E>
const std::vector<V>& Graph<V, E>::get_vertices() const{
    return vertices;
}

template<typename V, typename E>
Graph<V, E>::Graph(Graph<V, E> &&source): matrix(std::move(source.get_matrix())), vertices(std::move(source.get_vertices())),
edges(std::move(source.nrOfEdges())) {

}

template<typename V, typename E>
Graph<V, E>& Graph<V, E>::operator=(const Graph<V, E>&source) {
    matrix = source.get_matrix();
    vertices = source.get_vertices();
    edges = source.nrOfEdges();
    return *this;
}

template<typename V, typename E>
Graph<V, E>& Graph<V, E>::operator=(Graph<V, E> &&source) {
    matrix = std::move(source.get_matrix());
    vertices = std::move(source.get_vertices());
    edges = std::move(source.nrOfEdges());
    return *this;
}

template<typename V, typename E>
typename Graph<V, E>::VerticesIterator Graph<V, E>::insertVertex(const V &vertex_data) {
    vertices.push_back(vertex_data);
    matrix.push_back(std::vector<std::optional<E>>(vertices.size()));
    for(auto& row: matrix){
        row.push_back(std::optional<E>());
    }
    return Graph::VerticesIterator(&vertices, vertices.size() - 1);
}

template<typename V, typename E>
typename Graph<V, E>::VerticesIterator Graph<V, E>::removeVertex(std::size_t vertex_id) {
    if (vertex_id < 0 || vertex_id >= vertices.size()) return endVertices();
    for(std::size_t i = 0; i< matrix.size();i++){
        for(std::size_t j=0; j<matrix.size();j++){
            if(matrix[i][j].has_value() && (i==vertex_id || j==vertex_id)) {
                matrix[i][j].reset();
                edges--;
            }
        }
    }
    for(auto& row: matrix){
        row.erase(row.begin() + vertex_id);
    }
    matrix.erase(matrix.begin() + vertex_id);

    if(vertex_id == vertices.size()-1){
        vertices.pop_back();
        return endVertices();
    }
    vertices.erase(vertices.begin()+vertex_id);
    return VerticesIterator(&vertices, vertex_id);

}

template<typename V, typename E>
typename Graph<V, E>::ConstVerticesIterator Graph<V, E>::beginVertices() const{
    return Graph::ConstVerticesIterator(&vertices, 0);
}

template<typename V, typename E>
typename Graph<V, E>::ConstVerticesIterator Graph<V, E>::endVertices() const{
    return Graph::ConstVerticesIterator(&vertices, vertices.size());
}

template<typename V, typename E>
std::size_t Graph<V, E>::nrOfVertices() const {
    return vertices.size();
}

template<typename V, typename E>
void Graph<V, E>::printNeighborhoodMatrix() const {
    std::cout<<"Neighborhood Matix:\n";
    for(std::size_t i = 0; i < vertices.size(); i++){
        for(std::size_t j=0; j< vertices.size(); j++){
            if(matrix[i][j].has_value()) std::cout<<matrix[i][j].value()<<" ";
            else std::cout<<"X"<<" ";
        }
        std::cout<<std::endl;
    }
}

template<typename V, typename E>
typename Graph<V, E>::VerticesIterator Graph<V, E>::vertex(std::size_t vertex_id) {
    if (vertex_id < 0 || vertex_id >=vertices.size()) return endVertices();
    return Graph::VerticesIterator(&vertices, vertex_id);
}

template<typename V, typename E>
const V& Graph<V, E>::vertexData(std::size_t vertex_id) const {
    if (vertex_id < 0 || vertex_id >= vertices.size()) throw std::runtime_error("index out of range");
    return vertices[vertex_id];
}

template<typename V, typename E>
V& Graph<V, E>::vertexData(std::size_t vertex_id) {
    if (vertex_id < 0 || vertex_id >= vertices.size()) throw std::runtime_error("index out of range");
    return vertices[vertex_id];
}

template<typename V, typename E>
Graph<V, E>::~Graph() {

}

template<typename V, typename E>
typename Graph<V, E>::EdgesIterator Graph<V, E>::removeEdge(std::size_t vertex1_id, std::size_t vertex2_id) {
    if (vertex1_id < vertices.size() && vertex1_id >=0 && vertex2_id < vertices.size() && vertex2_id >=0) {
        if(!matrix[vertex1_id][vertex2_id].has_value()){
            return endEdges();
        } else{
            auto it = EdgesIterator(&matrix, vertex1_id, vertex2_id);
            matrix[vertex1_id][vertex2_id].reset();
            edges--;
            return it++;
        }
    }
    return endEdges();
}

template<typename V, typename E>
bool Graph<V, E>::edgeExist(std::size_t vertex1_id, std::size_t vertex2_id) const {
    if (vertex1_id < vertices.size() && vertex1_id >=0 && vertex2_id < vertices.size() && vertex2_id >=0){
        return matrix[vertex1_id][vertex2_id].has_value();
    }
    throw std::runtime_error("vertex_id out of range");
}

template<typename V, typename E>
std::pair<typename Graph<V, E>::EdgesIterator, bool>
Graph<V, E>::insertEdge(std::size_t vertex1_id, std::size_t vertex2_id, const E &label, bool replace) {
    if (vertex1_id < 0 || vertex1_id > vertices.size() || vertex2_id < 0 || vertex2_id > vertices.size()){
        return std::make_pair(endEdges(), false);
    }
    if(!matrix[vertex1_id][vertex2_id].has_value()) {
        matrix[vertex1_id][vertex2_id].emplace(label);
        edges++;
        return std::make_pair(EdgesIterator(&matrix, vertex1_id, vertex2_id), false);
    }
    if(matrix[vertex1_id][vertex2_id].has_value() && replace) {
        matrix[vertex1_id][vertex2_id].emplace(label);
        return std::make_pair(EdgesIterator(&matrix, vertex1_id, vertex2_id), true);
    }
    if(matrix[vertex1_id][vertex2_id].has_value() && !replace) {
        return std::make_pair(EdgesIterator(&matrix, vertex1_id, vertex2_id), false);
    }
}

template<typename V, typename E>
typename Graph<V, E>::ConstEdgesIterator Graph<V, E>::beginEdges() const{
    return Graph::ConstEdgesIterator(&matrix, 0);
}

template<typename V, typename E>
typename Graph<V, E>::ConstEdgesIterator Graph<V, E>::endEdges() const{
    return Graph::ConstEdgesIterator(&matrix, matrix.size());
}

template<typename V, typename E>
std::size_t Graph<V, E>::nrOfEdges() const {
    return edges;
}


template<typename V, typename E>
typename Graph<V, E>::EdgesIterator Graph<V, E>::edge(std::size_t vertex1_id, std::size_t vertex2_id) {
    if (vertex1_id < 0 || vertex2_id < 0 || vertex1_id >=vertices.size() || vertex2_id >=vertices.size())
        throw std::runtime_error("vertices indexes out of range");
    if(matrix[vertex1_id][vertex2_id].has_value()){
        return EdgesIterator(&matrix, vertex1_id, vertex2_id);
    }
    return endEdges();
}


template<typename V, typename E>
const E& Graph<V, E>::edgeLabel(std::size_t vertex1_id, std::size_t vertex2_id) const {
    if (vertex1_id < 0 || vertex2_id < 0 || vertex1_id >=vertices.size() || vertex2_id >=vertices.size())
        throw std::runtime_error("vertices indexes out of range");
    if(!matrix[vertex1_id][vertex2_id].has_value())
        throw std::runtime_error("there is no value");
    return matrix[vertex1_id][vertex2_id].value();
}

template<typename V, typename E>
E &Graph<V, E>::edgeLabel(std::size_t vertex1_id, std::size_t vertex2_id) {
    if (vertex1_id < 0 || vertex2_id < 0 || vertex1_id >=vertices.size() || vertex2_id >=vertices.size())
        throw std::runtime_error("vertices indexes out of range");
    if(!matrix[vertex1_id][vertex2_id].has_value())
        throw std::runtime_error("there is no value");
    return matrix[vertex1_id][vertex2_id].value();
}

template<typename V, typename E>
typename Graph<V, E>::VerticesIterator Graph<V, E>::beginVertices(){
    return Graph::VerticesIterator(&vertices, 0);
}

template<typename V, typename E>
typename Graph<V, E>::VerticesIterator Graph<V, E>::endVertices() {
    return Graph::VerticesIterator(&vertices, vertices.size());
}

template<typename V, typename E>
typename Graph<V, E>::ConstVerticesIterator Graph<V, E>::vertex(std::size_t vertex_id) const {
    if (vertex_id < 0 || vertex_id >=vertices.size()) return endVertices();
    return Graph::ConstVerticesIterator(&vertices, vertex_id);
}

template<typename V, typename E>
typename Graph<V, E>::ConstEdgesIterator Graph<V, E>::edge(std::size_t vertex1_id, std::size_t vertex2_id) const {
    if (vertex1_id < 0 || vertex2_id < 0 || vertex1_id >=vertices.size() || vertex2_id >=vertices.size())
        throw std::runtime_error("vertices indexes out of range");
    if(matrix[vertex1_id][vertex2_id].has_value()){
        return ConstEdgesIterator(&matrix, vertex1_id, vertex2_id);
    }
    return endEdges();
}

template<typename V, typename E>
typename Graph<V, E>::EdgesIterator Graph<V, E>::beginEdges() {
    return Graph::EdgesIterator(&matrix, 0);
}

template<typename V, typename E>
typename Graph<V, E>::EdgesIterator Graph<V, E>::endEdges() {
    return Graph::EdgesIterator(&matrix, matrix.size());
}

template<typename V, typename E>
typename Graph<V, E>::DFSIterator Graph<V, E>::beginDFS(std::size_t vertex_id) {
    return DFSIterator(this, vertex_id);
}

template<typename V, typename E>
typename Graph<V, E>::DFSIterator Graph<V, E>::endDFS() {
    return Graph::DFSIterator(this);
}

template<typename V, typename E>
typename Graph<V, E>::BFSIterator Graph<V, E>::beginBFS(std::size_t vertex_id) {
    return Graph::BFSIterator(this, vertex_id);
}

template<typename V, typename E>
typename Graph<V, E>::BFSIterator Graph<V, E>::endBFS() {
    return Graph::BFSIterator(this);
}

template <typename V, typename E>
void DFS(const Graph<V, E> &g, std::size_t start_id, std::function<void(const V&)> f){
    std::stack<std::pair<std::size_t, V>> st;
    std::vector<bool> visited(g.nrOfVertices(), false);
    st.push(std::make_pair(start_id, g.vertexData(start_id)));
    while(!st.empty()){
        auto& p = st.top();
        std::size_t id = p.first;
        V& v = p.second;
        st.pop();
        if(!visited[id]) {
            f(v);
            visited[id] = true;
        }
        auto it = typename Graph<V, E>::ConstEdgesIterator((&g.get_matrix()), id, 0);
        while(it && it.v1id() == id && it.v2id() < g.nrOfVertices()){
            if(!visited[it.v2id()]){
                st.push(std::make_pair(it.v2id(), g.vertexData(it.v2id())));
            }
            ++it;
        }
    }
}

template <typename V, typename E>
void BFS(const Graph<V, E> &g, std::size_t start_id, std::function<void(const V&)> f){
    std::vector<bool> visited(g.nrOfVertices(), false);
    std::queue<std::pair<std::size_t, V>> q;
    q.push(std::make_pair(start_id, g.vertexData(start_id)));
    while(!q.empty()){
        auto& p = q.front();
        std::size_t id = p.first;
        V& v = p.second;
        q.pop();
        if(!visited[id]) {
            f(v);
            visited[id] = true;
        }
        auto it = typename Graph<V, E>::ConstEdgesIterator((&g.get_matrix()), id, 0);
        while(it && it.v1id() == id && it.v2id() < g.nrOfVertices()){
            if(!visited[it.v2id()]){
                q.push(std::make_pair(it.v2id(), g.vertexData(it.v2id())));
            }
            ++it;
        }
    }
}

template <typename V, typename E>
std::pair<double, std::vector<std::size_t>> dijkstra(const Graph<V, E> &g, std::size_t start_id, std::size_t end_id, std::function<double(E)> to_double=nullptr){
    if(!to_double) to_double = [](E e){return e;};

    std::vector<std::optional<double>> distances(g.nrOfVertices());
    std::vector<std::optional<std::size_t>> previous(g.nrOfVertices());
    std::vector<bool> used(g.nrOfVertices(), false);

    distances[start_id].emplace(0.0);

    for(std::size_t i = 0; i<g.nrOfVertices() - 1; i++) {

        std::size_t id = 0;
        std::optional<double> min_dist = std::optional<E>();
            for (auto it = g.beginVertices(); it != g.endVertices(); it++) {
                if (!used[it.id()] && distances[it.id()].has_value()) {
                    if (!min_dist.has_value() || distances[it.id()].value() < min_dist.value()) {
                        min_dist = distances[it.id()];
                        id = it.id();
                    }
                }
            }

            used[id] = true;
            auto it = typename Graph<V, E>::ConstEdgesIterator((&g.get_matrix()), id, 0);
            while (it && it.v1id() == id && it.v2id() < g.nrOfVertices()) {
                if (!used[it.v2id()] && (!distances[it.v2id()].has_value() || distances[it.v2id()].value() > min_dist.value() + to_double(*it))) {
                    distances[it.v2id()].emplace(min_dist.value() + to_double(*it));
                    previous[it.v2id()].emplace(id);
                }
                ++it;
            }

    }

    std::vector<std::size_t> path;
    path.push_back(end_id);
    auto next = previous[end_id];
    while (next.has_value() && next.value() != start_id) {
        path.push_back(next.value());
        next = previous[next.value()];
    }
    path.push_back(start_id);
    std::reverse(path.begin(), path.end());
    double dist = 0.0;
    if(distances[end_id].has_value()) {
        dist =  distances[end_id].value();
    } else
        path = std::vector<std::size_t>();
    return std::make_pair(dist, path);

}

template<typename V, typename E>
std::pair<double, std::vector<std::size_t>> astar(const Graph<V, E> &g, std::size_t start_id, std::size_t end_id,
        const std::function<double(const Graph<V, E>&, const std::size_t&, const std::size_t&)>& h, std::function<double(E)> to_double=nullptr){

    if(!to_double) to_double = [](E e){ return e; };

    using p_type = std::pair<std::size_t, double>;
    //p_type.first == id
    //p_type.second == fScore[id]
    auto  comparator = [](p_type pr1, p_type pr2){return pr1.second > pr2.second; };
    std::priority_queue<p_type, std::vector<p_type>, decltype(comparator)> openSet(comparator);
    std::vector<std::optional<double>> gScore(g.nrOfVertices());   // the cheapest path
    std::vector<std::optional<double>> fScore(g.nrOfVertices());  //fScore[i] == gScore[i] + h[i]
    std::vector<std::optional<std::size_t>> cameFrom(g.nrOfVertices());  // the nodes that the path consists of
    std::vector<bool> inOpenSet(g.nrOfVertices(), false);

    gScore[start_id].emplace(0.0);
    openSet.push(std::make_pair(start_id, gScore[start_id].value()));
    inOpenSet[start_id] = true;
    fScore[start_id].emplace(h(g, start_id, end_id));

    while(!openSet.empty()) {
        auto &current = openSet.top();
        auto id = current.first;

        if (id == end_id) {
            std::vector<std::size_t> path;
            path.push_back(end_id);
            auto next = cameFrom[end_id];
            while (next.has_value() && next.value() != start_id) {
                path.push_back(next.value());
                next = cameFrom[next.value()];
            }
            path.push_back(start_id);
            std::reverse(path.begin(), path.end());
            return std::make_pair(gScore[end_id].value(), path);
        }

        openSet.pop();
        inOpenSet[id] = false;
        auto it = typename Graph<V, E>::ConstEdgesIterator((&g.get_matrix()), id, 0);
        while (it && it.v1id() == id && it.v2id() < g.nrOfVertices()) {

            if (gScore[id].has_value() && !gScore[it.v2id()].has_value() ||
                gScore[id].value() + to_double(*it) < gScore[it.v2id()].value()) {

                cameFrom[it.v2id()].emplace(id);
                gScore[it.v2id()].emplace(gScore[id].value() + to_double(*it));
                fScore[it.v2id()].emplace(gScore[it.v2id()].value() + h(g, it.v2id(), end_id));
                if (!inOpenSet[it.v2id()]) {
                    openSet.push(std::make_pair(it.v2id(), fScore[it.v2id()].value()));
                    inOpenSet[it.v2id()] = true;
                }
            }
            ++it;
        }
    }
    return std::make_pair(0.0, std::vector<std::size_t>());
}


#endif //ALGOS2_GRAPH_HPP
