import yaml
from types import SimpleNamespace
import os

class Config:
    """transform yaml file to class"""
    
    @staticmethod
    def load_from_yaml(file_path):
        if not os.path.exists(file_path):
            raise FileNotFoundError(f"File not found: {file_path}")
        
        with open(file_path, 'r', encoding='utf-8') as f:
            config_dict = yaml.safe_load(f)
        
        return Config._dict_to_object(config_dict)
    
    @staticmethod
    def _dict_to_object(d):
        if not isinstance(d, dict):
            return d
        
        obj = SimpleNamespace()
        for key, value in d.items():
            if isinstance(value, dict):
                setattr(obj, key, Config._dict_to_object(value))
            elif isinstance(value, list):
                setattr(obj, key, [Config._dict_to_object(item) if isinstance(item, dict) else item for item in value])
            else:
                setattr(obj, key, value)
        
        return obj