import rqt_graph.dotcode
from qt_dotgraph.pydotfactory import PydotFactory
import rosgraph.impl.graph


def get_current_rqt_graph_to_dotcode():
    dotcode_generator = rqt_graph.dotcode.RosGraphDotcodeGenerator()

    # starting at line 229 of ros_graph.py
    ns_filter = '/'
    topic_filter = '/'
    graph_mode = 'node_topic_all' # (Nodes/Topics(all))
    orientation = 'LR'
    namespace_cluster = 1

    accumulate_actions = True # (Group: (Actions))
    hide_dead_end_topics = False #True # (Dead sinks)
    hide_single_connection_topics = False #True (Leaf Topics)
    quiet = True
    dotcode_factory = PydotFactory()

    graph = rosgraph.impl.graph.Graph()
    graph.set_master_stale(5.0)
    graph.set_node_stale(5.0)
    graph.update()

    dotcode = dotcode_generator.generate_dotcode(
             rosgraphinst=graph,
             ns_filter=ns_filter,
             topic_filter=topic_filter,
             graph_mode=graph_mode,
             hide_single_connection_topics=hide_single_connection_topics,
             hide_dead_end_topics=hide_dead_end_topics,
             cluster_namespaces_level=namespace_cluster,
             accumulate_actions=accumulate_actions,
             dotcode_factory=dotcode_factory,
             orientation=orientation,
             quiet=quiet)

    return dotcode

def save_dotcode_to_file(file_path, dotcode):
    # save dotcode
    with open(file_path,'w+') as fHandle:
        fHandle.write(dotcode)
    fHandle.closed