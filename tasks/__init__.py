from .geometry.object_movement import ObjectMoveTask

def load_task(cfg):
    """
    Load the task based on the provided configuration and task name.
    
    Args:
        cfg (Config): The configuration object containing task settings.
        task_name (str): The name of the task to load.
    
    Returns:
        Task: An instance of the specified task.
    """
    # Import the task class dynamically
    task_type = cfg.tasks.type
    if task_type == "object_movement":
        return ObjectMoveTask(cfg)
    else:
        raise ValueError(f"Unsupported task type: {task_type}")