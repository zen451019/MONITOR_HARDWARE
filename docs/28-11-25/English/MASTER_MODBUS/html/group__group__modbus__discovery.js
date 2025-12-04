var group__group__modbus__discovery =
[
    [ "ModbusSensorParam", "struct_modbus_sensor_param.html", [
      [ "compressedBytes", "struct_modbus_sensor_param.html#a75de2e3796af5c5026559e921c645fb2", null ],
      [ "dataType", "struct_modbus_sensor_param.html#aa2671517f9085813612fd9bc23f39e18", null ],
      [ "maxRegisters", "struct_modbus_sensor_param.html#ac4b36098e6f0b3ad32493a2ba623ad8d", null ],
      [ "numberOfChannels", "struct_modbus_sensor_param.html#ad98ef02a850e274855fa238080343964", null ],
      [ "samplingInterval", "struct_modbus_sensor_param.html#ab1387b9ab39b6acbcc33ed206ae68b38", null ],
      [ "scale", "struct_modbus_sensor_param.html#abe04f837471525361d69b7a0e1d3565f", null ],
      [ "sensorID", "struct_modbus_sensor_param.html#a2ba74610b0ad901f3e5aef4c612fc3b4", null ],
      [ "startAddress", "struct_modbus_sensor_param.html#a05f0b0e1e6356bf737f21754af6f06e3", null ]
    ] ],
    [ "ModbusSlaveParam", "struct_modbus_slave_param.html", [
      [ "consecutiveFails", "struct_modbus_slave_param.html#af6a0cb55978342e2889a409b5f5132b5", null ],
      [ "sensors", "struct_modbus_slave_param.html#a3323379e7d93b8aef9d76cff0bbdb3d7", null ],
      [ "slaveID", "struct_modbus_slave_param.html#a937febe35c9d6eb970161b422bbe5c3d", null ]
    ] ],
    [ "ModbusRequestInfo", "struct_modbus_request_info.html", [
      [ "functionCode", "struct_modbus_request_info.html#a1a1681202a2033e7159e75f2531f014f", null ],
      [ "sensorId", "struct_modbus_request_info.html#a502153e9809f7780f74c8b95b71036a3", null ],
      [ "slaveId", "struct_modbus_request_info.html#a4c4a96aa5704784fb9139d5e7f4e0a8d", null ],
      [ "token", "struct_modbus_request_info.html#addd579a6c2d8d6a7d1e92db614348c4d", null ],
      [ "type", "struct_modbus_request_info.html#a2f735cf349ca7c974ed654bed2808455", null ]
    ] ],
    [ "ResponseFormat", "struct_response_format.html", [
      [ "data", "struct_response_format.html#a7922f4ea500627997a980f3915639caa", null ],
      [ "deviceId", "struct_response_format.html#ab8e9d3014aaaf03fa0cc05cf08c7e993", null ],
      [ "length", "struct_response_format.html#a17b3a5af96953469ab243485ecc82810", null ],
      [ "order", "struct_response_format.html#a9e5f636785ac2b8b300425802493d7bc", null ]
    ] ],
    [ "SensorSchedule", "struct_sensor_schedule.html", [
      [ "nextSampleTime", "struct_sensor_schedule.html#af5be7996637c19f353a650325f2d2380", null ],
      [ "samplingInterval", "struct_sensor_schedule.html#ae86b417e036040cc618400bc356437be", null ],
      [ "sensorID", "struct_sensor_schedule.html#a6ec60b6a64b6b8c80208905edd298dd7", null ],
      [ "slaveID", "struct_sensor_schedule.html#addc78eda4619408cacd1945c207daf2e", null ]
    ] ],
    [ "MAX_MODBUS_RESPONSE_LENGTH", "group__group__modbus__discovery.html#ga7f49ab031357b041f1760f2092028d70", null ],
    [ "RX_PIN", "group__group__modbus__discovery.html#gaa7089c01538b339ae745173c117f95b9", null ],
    [ "TX_PIN", "group__group__modbus__discovery.html#ga3bb17f5daa2b1eaef58c8aa2d989e27e", null ],
    [ "DiscoveryOrder", "group__group__modbus__discovery.html#ga46a07d73c79d4b8999372b819403da58", [
      [ "DISCOVERY_GET_COUNT", "main_8cpp.html#ga46a07d73c79d4b8999372b819403da58a1e1800af67b81d987bb00060533d5915", null ],
      [ "DISCOVERY_GET_DATA_OFFSET", "main_8cpp.html#ga46a07d73c79d4b8999372b819403da58a7a182e1d066bf8f45714fb1d7f0549e1", null ],
      [ "DISCOVERY_READ_SENSOR_PARAM", "main_8cpp.html#ga46a07d73c79d4b8999372b819403da58aea7e33cbb3a664bdb9a7e871353aa21e", null ]
    ] ],
    [ "RequestType", "group__group__modbus__discovery.html#gae10b07f2d0feb103db7fe4cfd192e5af", [
      [ "REQUEST_UNKNOWN", "main_8cpp.html#gae10b07f2d0feb103db7fe4cfd192e5afacc059c796c5ae2d0205ad730768999b7", null ],
      [ "REQUEST_DISCOVERY", "main_8cpp.html#gae10b07f2d0feb103db7fe4cfd192e5afa3232d13dce2d61800581b3b1c0b1611d", null ],
      [ "REQUEST_SAMPLING", "main_8cpp.html#gae10b07f2d0feb103db7fe4cfd192e5afa5ade4fdadde8b1ec2bc10668653efa7b", null ]
    ] ],
    [ "addRequest", "group__group__modbus__discovery.html#ga2b6bd4e110caa560464cc252a7dc787f", null ],
    [ "DataRequestScheduler", "group__group__modbus__discovery.html#gafe37cc62bf5e1b7be5d4b906edac7444", null ],
    [ "discoverDeviceSensors", "group__group__modbus__discovery.html#ga86b51577aeff55acd47a303f408339ea", null ],
    [ "EventManager", "group__group__modbus__discovery.html#gac0c8aeee251d675a0300a1e3be22e9fe", null ],
    [ "findRequestByToken", "group__group__modbus__discovery.html#ga5e9b156cfc98d9c060bfd23770ed3590", null ],
    [ "getRegistersPerChannel", "group__group__modbus__discovery.html#ga8a81627d82ccf43aef00a4023d2d6e55", null ],
    [ "getSensorParams", "group__group__modbus__discovery.html#ga320e20ed89286bb200de38e3c0d74cde", null ],
    [ "handleData", "group__group__modbus__discovery.html#gae13ecbdf143b216f696e59a9b7edb141", null ],
    [ "handleError", "group__group__modbus__discovery.html#ga944f73ca5a3437229e84a3317d2a1634", null ],
    [ "initialDiscoveryTask", "group__group__modbus__discovery.html#gab0e674715eff7261b0ae3a35fe239be4", null ],
    [ "initScheduler", "group__group__modbus__discovery.html#gaec0c4943af093ad42d2d099ceeadb98c", null ],
    [ "parseAndStoreDiscoveryResponse", "group__group__modbus__discovery.html#gacd852b44946c142cee5128d699ead65a", null ],
    [ "MB", "group__group__modbus__discovery.html#gafba4fdd67a5d79a0cdbf6d3652ccb5b5", null ],
    [ "queueEventos_Peripheral", "group__group__modbus__discovery.html#gab204d0e5b1e10f59187110e07d7782b4", null ],
    [ "queueRespuestas", "group__group__modbus__discovery.html#gaa744737f20ae412448361b8aa57d8efb", null ]
];