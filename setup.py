from setuptools import setup, find_packages
import sys, os.path
import pandas as pd

from glob import glob

if __name__ == "__main__":
      print("Searching for esailor virtual env...")

      paths2env = []
      home      = os.path.expanduser("~")

      for dir, _, _ in os.walk(os.path.join(home,"miniconda3")):
            paths2env.extend(glob(os.path.join(dir, "*/esailor/bin/python")))

      if len(paths2env) == 0:
            print("Miniconda not found! Searching for alternatives...")
            for dir, _, _ in os.walk(home):
                  paths2env.extend(glob(os.path.join(dir, "*/envs/esailor/bin/python")))
            if len(paths2env) == 0:
                  for dir, _, _ in os.walk("/"):
                        paths2env.extend(glob(os.path.join(dir, "*/envs/esailor/bin/python")))

      if len(paths2env) > 0:
            path2env = paths2env[0]
            print(f"Environment found: {path2env}")
      else:
            print("Python virtual environment esailor not found!\n")
            user_opt = ""
            count = 0
            while (user_opt != "y") & (user_opt != "n") & (count < 3):
                  user_opt = input("Do you want to install the environment [y, n]: ")
                  count += 1
            
            if user_opt == "y":
                  print("Searching for the miniconda HOME directory...")
                  paths2miniconda = []
                  for dir, _, _ in os.walk(home):
                        paths2miniconda.extend(glob(os.path.join(home, "miniconda*")))

                  df = pd.DataFrame(paths2miniconda, columns=["path"])
                  df.drop_duplicates(keep='first', inplace=True, ignore_index=False)
                  paths2miniconda = df.path.values
                  del(df)

                  num_of_paths = len(paths2miniconda)
                  if num_of_paths == 0:
                        for dir, _, _ in os.walk(home):
                              paths2miniconda.extend(glob(os.path.join(home, "*/miniconda*")))
                  #-------------------------------------------------------------
                  if num_of_paths == 0:
                        print("Miniconda installation not found!")
                        path2miniconda = input("Manualy enter the path (or type return for aborting setup.py): ")
                  elif num_of_paths > 1:
                        print("Multiple similar paths found!")
                        for i, pathfound in enumerate(paths2miniconda):
                              print(f"{i} - {pathfound}")
                        pathid = input("\nChose the desired instalation path: ")
                        path2miniconda = paths2miniconda[pathid]
                  else:
                        path2miniconda = paths2miniconda[0]
                        print(f"Miniconda installation found at {path2miniconda}")
                  #-------------------------------------------------------------
                  if len(path2miniconda) == 0:
                        print("Aborting setup")
                        exit(1)
                  else:
                        path2env = os.path.join(path2miniconda,"envs/esailor2")
                        file = "esailor.yml"
                        with open(file, 'r') as f:
                              while True:
                                    line = f.readline()
                                    if "/envs/esailor" in line:
                                          path2replace = line.split(":")[1]
                                    if not line:
                                          break

                        with open(file, 'r') as f:
                              filedata = f.read()
                        filedata = filedata.replace(path2replace, " "+path2env+"\n")
                        with open(file, 'w') as f:
                              f.write(filedata)

                        os.system("conda env create -f esailor.yml")
                        path2env = os.path.join(path2env,"bin/python")

            elif user_opt == "n":
                  print("Provide the path to the python interpreter (or type return for aborting setup.py):")
                  path2env = input()
                  if len(path2env) == 0:
                        print("Aborting setup")
                        exit(1)
            else:
                  print("Aborting setup")
                  exit(1) 

      print("Searching for files that require update...")

      files     = []
      start_dir = os.getcwd()
      pattern   = "*.py"
      for dir, _, _ in os.walk(start_dir):
            files.extend(glob(os.path.join(dir,pattern)))

      a = os.path.join(os.getcwd(), "setup.py")
      files.remove(os.path.join(os.getcwd(), "setup.py"))

      for file in files:
            with open(file, 'r') as f:
                  line = f.readline()
            if "/envs/esailor/bin/python" in line:
                  path2replace = line.split("!")[1]

                  with open(file, 'r') as f:
                        filedata = f.read()
                  filedata = filedata.replace(path2replace, path2env+"\n")
                  with open(file, 'w') as f:
                        f.write(filedata)