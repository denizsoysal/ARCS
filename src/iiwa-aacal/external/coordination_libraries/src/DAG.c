/*
 * @file DAG.c
 * @brief Direct Acyclic Graph Implementation
 *
 * Direct Acyclic Graph implementation using CGraph
 * (Edges named after the number of totol outgoing edges of tail node)
 *
 * (c) Filip Reniers (KU Leuven) 19.03.20
 *
 */
#include <coordination-libs/DAG/DAG.h>

DAG_t *init_DAG(char *DAG_name) {
    DAG_t *g = agopen(DAG_name, Agdirected, NULL);
    agattr(g, AGNODE, "type", "");
    return g;
}

void destroy_DAG(DAG_t *g) {
    agclose(g);
}

vertex_t *create_vertex(DAG_t *g, char *vertex_name) {
    vertex_t *p1 = agnode(g, vertex_name, TRUE);
    return p1;
}

vertex_t *create_vertex_with_type(DAG_t *g, char *vertex_name, char* type) {
    vertex_t *p1 = agnode(g, vertex_name, TRUE);
    agset(p1, "type", type);
    return p1;
}

char *get_vertex_name(vertex_t *v){
    return agnameof(v);
}

vertex_t *get_vertex_by_name(DAG_t *g, char *vertex_name){
    return agnode(g, vertex_name, FALSE);
}

void destroy_vertex(DAG_t *g, char *vertex_name) {
    vertex_t *p1 = get_vertex_by_name(g, vertex_name);
    agdelnode(g, p1);
}

void set_vertex_type(DAG_t *g, char *vertex_name, char *type){
    vertex_t *p1 = get_vertex_by_name(g, vertex_name);
    agset(p1, "type", type);
}


char *get_vertex_type(vertex_t *v){
    return agget(v, "type");
}

char *get_vertex_type_by_name(DAG_t *g, char *name){
    vertex_t *p1 = get_vertex_by_name(g, name);
    return agget(p1, "type");
}

int get_number_of_outgoing_edges(vertex_t *p){
    return agdegree(agraphof(p),p, FALSE, TRUE);
}

void create_connection_vertices(DAG_t *g, vertex_t *parent, vertex_t *child) {

    int n_outgoing_edges_parent = get_number_of_outgoing_edges(parent);
    char buffer[8];
    sprintf(buffer, "%d",n_outgoing_edges_parent);
    agedge(g, parent, child, buffer, TRUE);
}

void create_connection_vertices_by_name(DAG_t *g, char *parent_vertex_name, char *child_vertex_name) {
    vertex_t *parent = get_vertex_by_name(g, parent_vertex_name);
    vertex_t *child = get_vertex_by_name(g, child_vertex_name);

    create_connection_vertices(g, parent, child);
}

Agedge_t *get_outgoing_edge_of_named_vertex_by_number(DAG_t *g, char *parent_vertex_name, int outgoing_edge_number ){
    vertex_t *parent = get_vertex_by_name(g, parent_vertex_name);
    char buffer[8];
    sprintf(buffer, "%d",outgoing_edge_number);

    for (Agedge_t *e = agfstout(g,parent); e; e = agnxtout(g,e)){

        if (!strcmp(agnameof(e), buffer)){
            return e;
        }
    }
    return NULL;
}

Agedge_t *get_outgoing_edge_by_number(vertex_t *parent, int outgoing_edge_number ){
    char buffer[8];
    sprintf(buffer, "%d",outgoing_edge_number);

    for (Agedge_t *e = agfstout(agraphof(parent),parent); e; e = agnxtout(agraphof(parent),e)){

        if (!strcmp(agnameof(e), buffer)){
            return e;
        }
    }
    return NULL;
}

vertex_t *next_first_vertex(vertex_t *v){
    Agedge_t *e = get_outgoing_edge_by_number(v, 0 );
    return aghead(e);
}

vertex_t *next_second_vertex(vertex_t *v){
    Agedge_t *e = get_outgoing_edge_by_number(v, 1);
    return aghead(e);
}

void destroy_connection_vertex(DAG_t *g, char *parent_vertex_name, int outgoing_edge_number){ // todo the name assignment is not correct anymore after deleting an edge
    Agedge_t *e = get_outgoing_edge_of_named_vertex_by_number(g, parent_vertex_name,outgoing_edge_number);
    agdeledge(g, e);
}

void fprint_DAG(DAG_t *g, FILE *pipe) {
    fprintf(pipe, "\n");
    for (Agnode_t *n = agfstnode(g); n; n = agnxtnode(g, n)) {
            fprintf(pipe, "vertex %s has type %s:\n", agnameof(n), agget(n, "type"));
            fprintf(pipe, "number of outgoing edges: %d\n", agdegree(g, n, FALSE, TRUE));
            for( Agedge_t *e = agfstout(g,n); e; e = agnxtout(g,e)) {
                Agnode_t *n_child = aghead(e);
                fprintf(pipe, "Vertex %s -> Edge %s -> Vertex %s:\n", agnameof(n), agnameof(e), agnameof(n_child));
            }
    }
    fprintf(pipe, "\n");
}

void print_DAG(DAG_t *g) {
    fprint_DAG(g, stdout);
}

static char node1[8] = "node1";
static char node2[8] = "node2";
static char node3[8] = "node3";
static char node4[8] = "node4";

void test_DAG(){

    DAG_t *dag = init_DAG("My First DAG");
    create_vertex(dag, node1);
    create_vertex(dag, node2);
    create_vertex(dag, node3);
    create_vertex(dag, node4);

    create_connection_vertices_by_name(dag, node1, node2);
    create_connection_vertices_by_name(dag, node1, node3);
    create_connection_vertices_by_name(dag, node1, node4);
    create_connection_vertices_by_name(dag, node2, node4);
    create_connection_vertices_by_name(dag, node3, node4);
    create_connection_vertices_by_name(dag, node3, node2);

    set_vertex_type(dag, node1, "process_node");
    set_vertex_type(dag, node2, "decision_node");
    set_vertex_type(dag, node3, "process_node");
    set_vertex_type(dag, node4, "process_node");

    vertex_t *v = get_vertex_by_name(dag, node2);

    print_DAG(dag);

    destroy_DAG(dag);



}

void create_vertex_attribute_DAG(vertex_t *v, void *attribute) {
     vertex_attribute_t *rec = (vertex_attribute_t *) agbindrec(v, "attribute",
                                                                       sizeof(vertex_attribute_t), 0);
    if (rec) {
        rec->attr = attribute;
    }
}


void destroy_vertex_attribute_DAG(vertex_t *t1) {
    vertex_attribute_t *rec = NULL;
    if (rec = (vertex_attribute_t *) aggetrec(t1, "attribute", 0)) {
        agdelrec(t1, "attribute");
    }
}

void *get_vertex_attribute(vertex_t *v) {
    vertex_attribute_t *rec = NULL;
    rec = (vertex_attribute_t *) aggetrec(v, "attribute", 0);
    if (rec)
        return rec->attr;
    else
        return NULL;
}














