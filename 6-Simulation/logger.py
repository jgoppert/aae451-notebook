import pandas as pd

class Logger:
    
    def __init__(self, topics=None):
        self.time = []
        self.data = {}
        self.topics = topics

    def new_frame(self, index, data):
        self.time.append(index)
        if self.topics is not None:
            save_keys = self.topics
        else:
            save_keys = data.keys()
        for key in save_keys:
            if key not in self.data.keys():
                self.data[key] = []
            self.data[key].append(data[key])
    
    def to_pandas(self):
        return pd.DataFrame(self.data, pd.Index(self.time, name='t, sec'))
