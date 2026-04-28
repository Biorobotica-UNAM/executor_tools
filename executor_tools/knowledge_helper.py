from executor_interfaces.srv import UpdateKnowledge

def save_to_knowledge(node, label, value):
    """
    Envía una petición al Worker para actualizar la RAM y el YAML.
    :param node: El nodo de ROS 2 desde donde se llama (self)
    :param label: El nombre del label (ej: 'name_1')
    :param value: El valor a guardar (ej: 'jorge')
    """
    # Creamos el cliente
    client = node.create_client(UpdateKnowledge, '/knowledge/update_label')
    
    # Espera rápida para no congelar el nodo
    if not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().warn(f"Service /knowledge/update_label not available for label: {label}")
        return None

    req = UpdateKnowledge.Request()
    req.label = label
    req.value = str(value)

    # Retorna el 'future' para que el nodo sepa que se mandó
    return client.call_async(req)