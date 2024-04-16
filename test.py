##todo
import casadi as ca
import pickle

#Load the lookup table from the file
with open("lookup_table2.pkl", "rb") as f:
    lookup_table_loaded = pickle.load(f)

state = [10,2,0,5,6]
# Evaluate the lookup table at a specific value
print(lookup_table_loaded(state))                  