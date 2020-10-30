import subprocess
import sys
import os
from pathlib import Path
import yaml
import tempfile
import copy

tesseract_support_dir = Path(os.environ["TESSERACT_SUPPORT_DIR"])

urdf_filename = tesseract_support_dir.joinpath("urdf").joinpath("ibr_iiwa_14_r820.urdf")
srdf_filename = tesseract_support_dir.joinpath("urdf").joinpath("ibr_iiwa_14_r820.urdf")

buckets_info = {"buckets": [{"name": "tesseract_support", "directory": tesseract_support_dir}]}

node_env = copy.deepcopy(os.environ)

with tempfile.NamedTemporaryFile() as buckets_f:
    yaml.dump(buckets_info, buckets_f)
    node_env["ROBOTRACONTEUR_BUCKET_FILES"] = buckets_f.name

    subprocess.check_call("tesseract_robotraconteur_service", f"--urdf-file={urdf_filename}", f"--srdf-file={srdf_filename}")



