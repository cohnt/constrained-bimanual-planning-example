import os

def RepoDir():
    return os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
