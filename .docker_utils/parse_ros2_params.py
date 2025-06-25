import yaml
import sys

def flatten_dict(d, parent_key=''):
    items = {}
    for k, v in d.items():
        new_key = f"{parent_key}/{k}" if parent_key else k
        if isinstance(v, dict):
            items.update(flatten_dict(v, new_key))
        else:
            items[new_key] = v
    return items

def transform_yaml(input_path, output_path):
    with open(input_path, 'r') as f:
        data = yaml.safe_load(f)

    transformed = {}
    for top_key, val in data.items():
        transformed[top_key] = {
            'ros__parameters': flatten_dict(val) if isinstance(val, dict) else val
        }

    with open(output_path, 'w') as f:
        yaml.dump(transformed, f, sort_keys=False)

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <input_yaml_file> <output_yaml_file>", file=sys.stderr)
        sys.exit(1)
    input_file = sys.argv[1]
    output_path = sys.argv[2]
    transform_yaml(input_file, output_path)
