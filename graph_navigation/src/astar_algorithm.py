#!/usr/bin/env python3

import heapq
import numpy as np

#Euristica usata da A* --> distanza euclidea
def heuristic(node1, node2):
    return np.linalg.norm(np.array(node1) - np.array(node2))

#Estrazione del path minimo
def astar(graph, start, goal):
    #Definizione heap di nodi da valutare
    exploring_set = []
    heapq.heappush(exploring_set, (0.0, start))

    #Dizionario utile a ricostruire il percorso
    dictionary_nodes = {}
    g_cost = {start: 0.0}

    #set() --- > Utile per capire se un nodo appartiene a quelli già visitati
    #Complessità computazionale ridotta notevolmente per la ricerca
    explored_set = set()

    #Finchè c'è qualcosa nel set da controllare vado avanti col codice
    while exploring_set:
        # Estrazione primo elemento dell' heap
        _, current_node = heapq.heappop(exploring_set)

        #Ricostruzione del percorso in caso di raggiungimento del goal
        if current_node == goal:
            return reconstruct_path(dictionary_nodes, current_node)

        #Il nodo corrente lo sto valutando e quindi lo metto nel set closed
        explored_set.add(current_node)

        #Esploro i vicini del nodo corrente e prendo il costo associato all'arco che li collega
        for neighbor, edge_cost in graph.get(current_node, []):

            if neighbor in explored_set:
                continue

            #calcolo del costo attuale + eventuale costo del vicino preso in considerazione
            current_cost = g_cost[current_node] + edge_cost

            #S il vicino non è in g_cost(non valutato) oppure il costo calcolato è piu piccolo di quello che già c'e
            # allora si aggiunge il nodo corrente al dizionario e gli si associa il costo
            # Se già c'era invece gli viene sostituito il valore
            if neighbor not in g_cost or current_cost < g_cost[neighbor]:
                dictionary_nodes[neighbor] = current_node
                g_cost[neighbor] = current_cost
                f = current_cost + heuristic(neighbor, goal)

                heapq.heappush(exploring_set, (f, neighbor))

    #Restituisce un array vuoto se non c'è un percorso
    return []

#Funzione che ricostruisce il path tenendo in considerazione il dizionario generato in precedenza e il current node
def reconstruct_path(dictionary_nodes, current_node):
    path = [current_node]
    while current_node in dictionary_nodes:
        current_node = dictionary_nodes[current_node]
        path.append(current_node)
    path.reverse()
    return path
