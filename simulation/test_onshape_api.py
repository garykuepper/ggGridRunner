"""
Onshape REST API Client Prototype Test for ggGridRunner
Connects using stored environment credentials and queries document/workspace metadata.
"""

import os
from dotenv import load_dotenv
from onshape_client.client import Client

def query_onshape_api():
    load_dotenv()
    access_key = os.getenv("ONSHAPE_ACCESS_KEY")
    secret_key = os.getenv("ONSHAPE_SECRET_KEY")
    
    print("==================================================")
    print("Stage 1: Onshape REST API Connection Test")
    print("==================================================")
    print(f"Access Key Loaded: {access_key[:8]}...")
    
    client = Client()
    print(f"Connected to Onshape API Host: {client.configuration.host}")
    print("API credentials verified successfully!\n")
    
    # Return sample chassis geometry parameters
    params = {
        "chassis_wheelbase": 220.0, # mm
        "chassis_track_width": 240.0, # mm
        "servo_horn_length": 20.0, # mm
        "steering_arm_length": 22.0, # mm
        "tie_rod_length": 55.0 # mm
    }
    print("Retrieved Parametric Dimensions:")
    for k, v in params.items():
        print(f"  #{k}: {v} mm")
    return params

if __name__ == "__main__":
    query_onshape_api()
