/*
 * @file DAG.h
 * @brief Header of Direct Acyclic Graph Implementation
 *
 * Direct Acyclic Graph implementation using CGraph
 * (Edges named after the number of totol outgoing edges of tail node)
 *
 * (c) Filip Reniers (KU Leuven) 19.03.20
 *
 */

#ifndef COORDINATION_LIBS_DAG_H
#define COORDINATION_LIBS_DAG_H

#include <stdbool.h>
#include <assert.h>
#include <time.h>
#include <stdlib.h>
#include <cgraph.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef Agnode_t vertex_t;
typedef Agraph_t DAG_t;

typedef struct vertex_attribute_s {
    Agrec_t rec;
    void *attr;
} vertex_attribute_t;

/* DAG CREATION FUNCTIONS */
DAG_t *init_DAG(char *DAG_name);
void destroy_DAG(DAG_t *g);

vertex_t *create_vertex(DAG_t *g, char *vertex_name);
vertex_t *create_vertex_with_type(DAG_t *g, char *vertex_name, char* type);
void destroy_vertex(DAG_t *g, char *vertex_name);

void create_connection_vertices(DAG_t *g, vertex_t *parent, vertex_t *child);
void create_connection_vertices_by_name(DAG_t *g, char *parent_vertex_name, char *child_vertex_name);
void destroy_connection_vertex(DAG_t *g, char *parent_vertex_name, int outgoing_edge_number);

/* DAG QUERY FUNCTIONS */
char *get_vertex_name(vertex_t *v);
vertex_t *get_vertex_by_name(DAG_t *g, char *vertex_name);
Agedge_t *get_outgoing_edge_of_named_vertex_by_number(DAG_t *g, char *parent_vertex_name, int outgoing_edge_number );
Agedge_t *get_outgoing_edge_by_number(vertex_t *parent, int outgoing_edge_number);
vertex_t *next_first_vertex(vertex_t *v);
vertex_t *next_second_vertex(vertex_t *v);

char *get_vertex_type(vertex_t *v);
char *get_vertex_type_by_name(DAG_t *g, char *name);
void set_vertex_type(DAG_t *g, char *vertex_name, char *type);

int get_number_of_outgoing_edges(vertex_t *p);

/* DAG VISUALIZATION FUNCTIONS */
void fprint_DAG(DAG_t *g, FILE *pipe);
void print_DAG(DAG_t *g);

void test_DAG();

/* DAG vertex attributes */
void create_vertex_attribute_DAG(vertex_t *v, void *attribute);
void destroy_vertex_attribute_DAG(vertex_t *t1);
void *get_vertex_attribute(vertex_t *v);

#ifdef __cplusplus
}
#endif

#endif //COORDINATION_LIBS_DAG_H
