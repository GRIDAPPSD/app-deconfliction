from argparse import ArgumentParser

from cimgraph.data_profile import rc4_2021 as cim
from cimgraph.databases import ConnectionParameters
from cimgraph.databases.blazegraph.blazegraph import BlazegraphConnection
from cimgraph.models import FeederModel
import cimgraph.utils as cimUtils
import importlib


if __name__ == '__main__':
    mrid = '_E3D03A27-B988-4D79-BFAB-F9D37FB289F7'
    db_url = 'http://localhost:8889/bigdata/namespace/kb/sparql'
    params = ConnectionParameters(url=db_url, cim_profile='rc4_2021', iec61970_301=7)
    db_connection = BlazegraphConnection(params)
    feeder_container = cim.Feeder(mRID=mrid)
    cim_model = FeederModel(connection=db_connection, container=feeder_container, distributed=False)
    cimUtils.get_all_data(cim_model)
    cimUtils.write_xml(cim_model, 'ieee123apps_with_measurements.xml')