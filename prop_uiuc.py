from pathlib import Path
import pandas as pd

def parse_database(verbose=True):
    database = {}
    for path in [Path('UIUC-propDB/volume-1/data'), Path('UIUC-propDB/volume-2/data')]:
        for file in path.glob('*.txt'):
            args = file.stem.split('_')
            try:
                man = args[0]
                D, pitch = args[1].split('x')
                if len(args) > 2:
                    key = "{:s}_{:s}x{:s}".format(man, D, pitch)
                    if key not in database.keys():
                        database[key] = {
                            'man': man,
                            'D': float(D),
                            'pitch': float(pitch),
                            'static': None,
                            'geom': None,
                            'dynamic': {}
                        }
                data = pd.read_csv(file, index_col=0, sep='\s+');
                if args[2] == 'static':
                    database[key]['static'] = {
                        'label': args[3],
                        'data': data
                    }
                elif args[2] == 'geom':
                    database[key]['geom'] = data
                elif len(args) == 4:
                    rpm = args[3]
                    database[key]['dynamic'][float(rpm)] = {
                        'label': args[2],
                        'data': data
                    }
            except Exception as e:
                if verbose:
                    print('non-standard format', file.stem, e)
                continue
    return database
