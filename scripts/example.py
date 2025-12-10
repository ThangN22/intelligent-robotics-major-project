import matrix
import node
import plan_path as path

node_0 = node.Node()
node_1 = node.Node()
node_2 = node.Node()
node_3 = node.Node()
node_4 = node.Node()
node_5 = node.Node()
node_6 = node.Node()

node_0.create_node((10.0, 12.5))
node_1.create_node((10.0, 19.0))
node_2.create_node((15.0, 10.0))
node_3.create_node((10.0, 7.5,))
node_4.create_node((10.0, 22.5))
node_5.create_node((15, 22.5))
node_6.create_node((6.0, 22.5))

node_list = (node_0, node_1, node_2, node_3, node_4, node_5, node_6)

mat = matrix.Matrix()

mat.create_matrix(len(node_list))

mat.add_node_list(node_list)




mat.print_node_list()

mat.add_edge(0,1)
mat.add_edge(0, 2)
mat.add_edge(0, 3)
mat.add_edge(1, 4)
mat.add_edge(4, 5)
mat.add_edge(4, 6)

mat.print()

mat.print_simple()

node_3_to_node_5 = path.a_star_search(node_3, node_5, node_list, mat.mat)

path.print_a_star_results(node_3_to_node_5, node_list)

