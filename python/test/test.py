import os


def getDictValue(dict, key):
  try:
    if type(key) == list:
      result = dict
      for k in key:
        result = result[k]
      return result
    return dict[key]
  except:
    return ""


a = {"a": {"b": 1}}
print(getDictValue(a, ["a", "b"]))
