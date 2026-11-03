// 🔹 Constants
const CONSTANTS = {
  LENGTH_MASK: 0x1f,
  CHANNEL_COUNT: 3,
  CURRENT_BITS_PER_VALUE: 10,
  BITS_PER_BYTE: 8,
};

const SENSOR_CONFIG = {
  0: {
    name: "energy",
    dataSize: "4_bytes", // Each value = 1 byte
    channels: 1,
    frecuency: 30000, // 300ms between measurements
    conversion: (val) => val * 0.1,
    valueType: "int", // float | int | uint
  },
  1: {
    name: "voltage",
    dataSize: "4_bytes", // Each value = 10 bits packed
    channels: 3,
    frecuency: 30000, // 300ms between measurements
    conversion: (val) => val * 0.1,
    valueType: "int",
  },
  2: {
    name: "current",
    dataSize: "4_bytes", // Each value = 10 bits packed
    channels: 3,
    frecuency: 30000, // 300ms between measurements
    conversion: (val) => val * 0.01,
    valueType: "int",
  },
  3: {
    name: "real_power",
    dataSize: "4_bytes", // Each value = 2 bytes
    channels: 1,
    frecuency: 30000, // 300ms between measurements
    conversion: (val) => val * 0.1,
    valueType: "int",
  },
  4: {
    name: "apparent_power",
    dataSize: "4_bytes",
    channels: 1,
    frecuency: 30000, // 300ms between measurements
    conversion: (val) => val * 0.1,
    valueType: "int",
  },
  5: {
    name: "reactive_power",
    dataSize: "4_bytes",
    channels: 1,
    frecuency: 30000, // 300ms between measurements
    conversion: (val) => val * 0.1,
    valueType: "int",
  },
  6: {
    name: "power_factor",
    dataSize: "4_bytes",
    channels: 1,
    frecuency: 30000, // 300ms between measurements
    // Register 0x0027: High byte = Phase C, Low byte = Combined (LSB = 0.01).
    conversion: (val) => (val & 0xff) * 0.01,
    valueType: "int",
  },
  7: {
    name: "frequency",
    dataSize: "4_bytes",
    channels: 1,
    frecuency: 30000, // 300ms between measurements
    conversion: (val) => val * 0.01,
    valueType: "int",
  },
};

// 🔹 Utility Functions
function validateInput(buffer) {
  if (!buffer || buffer.length < 5) {
    throw new Error("Invalid payload: insufficient data length");
  }
}

function isSensorActive(activationByte, sensorBit) {
  return (activationByte >> sensorBit) & 1;
}

function getActiveSensors(activationByte) {
  const activeSensors = [];
  for (let bit = 0; bit < 8; bit++) {
    if (isSensorActive(activationByte, bit)) {
      activeSensors.push({
        bit,
        name: SENSOR_CONFIG[bit].name,
        config: SENSOR_CONFIG[bit],
      });
    }
  }
  return activeSensors;
}

// 🔹 Data Processing by Size
function processByDataSize(
  rawBytes,
  dataSize,
  valueCount,
  conversion,
  valueType
) {
  let values = [];

  switch (dataSize) {
    case "1_byte":
      values = Array.from(rawBytes).slice(0, valueCount);
      break;

    case "2_bytes":
      for (
        let i = 0;
        i < rawBytes.length && values.length < valueCount;
        i += 2
      ) {
        if (i + 1 < rawBytes.length) {
          const value = (rawBytes[i] << 8) | rawBytes[i + 1];
          values.push(value);
        }
      }
      break;

    case "4_bytes": {

      const view = new DataView(
        rawBytes.buffer,
        rawBytes.byteOffset,
        rawBytes.byteLength
      );

      for (
        let i = 0;
        i + 3 < rawBytes.length && values.length < valueCount;
        i += 4
      ) {
        switch (valueType) {
          case "float":
            values.push(view.getFloat32(i, false));
            break;
          case "uint":
            values.push(view.getUint32(i, false));
            break;
          case "int":
          default:
            values.push(view.getInt32(i, false));
            break;
        }
      }

      break;
    }

    case "10_bits":
      values = convertPackedBitsToValues(
        rawBytes,
        valueCount,
        CONSTANTS.CURRENT_BITS_PER_VALUE
      );
      break;

    default:
      throw new Error(`Unsupported data size: ${dataSize}`);
  }

  return values.map(conversion);
}

function convertPackedBitsToValues(rawBytes, valueCount, bitsPerValue) {
  // Para datos de corriente de 10 bits, necesitamos un enfoque diferente
  // Los valores de 10 bits están empaquetados de manera específica

  const values = [];
  let bitOffset = 0;

  for (let i = 0; i < valueCount; i++) {
    let value = 0;

    // Extraer 10 bits desde la posición bitOffset
    for (let bit = 0; bit < bitsPerValue; bit++) {
      const byteIndex = Math.floor((bitOffset + bit) / 8);
      const bitIndex = (bitOffset + bit) % 8;

      if (byteIndex < rawBytes.length) {
        const bitValue = (rawBytes[byteIndex] >> (7 - bitIndex)) & 1;
        value = (value << 1) | bitValue;
      }
    }

    values.push(value);
    bitOffset += bitsPerValue;
  }

  return values;
}

// 🔹 Channel Distribution
function distributeToChannels(values, channelCount) {
  if (channelCount === 1) {
    return { ch1: values };
  }

  const valuesPerChannel = Math.floor(values.length / channelCount);
  const channels = {};

  for (let ch = 0; ch < channelCount; ch++) {
    const startIndex = ch * valuesPerChannel;
    const endIndex = startIndex + valuesPerChannel;
    channels[`ch${ch + 1}`] = values.slice(startIndex, endIndex);
  }

  return channels;
}

// 🔹 Channel Distribution for Timestamped Data
function distributeTimestampedToChannels(timestampedValues, channelCount) {
  if (channelCount === 1) {
    return { ch1: timestampedValues };
  }

  const valuesPerChannel = Math.floor(timestampedValues.length / channelCount);
  const channels = {};

  for (let ch = 0; ch < channelCount; ch++) {
    const startIndex = ch * valuesPerChannel;
    const endIndex = startIndex + valuesPerChannel;
    channels[`ch${ch + 1}`] = timestampedValues.slice(startIndex, endIndex);
  }

  return channels;
}

// 🔹 Format Data with Timestamps
function formatDataWithTimestamps(values, arrivalTime, frequency) {
  return values.map((value, index) => ({
    value: value,
    time: new Date(arrivalTime + (index * frequency)).toISOString(),
  }));
}

// 🔹 Header Processing
function parseHeader(buffer) {
  let position = 0;

  const messageId = buffer[position++];
  const timestamp = readUInt32BE(buffer, position);
  position += 4;
  const activationByte = buffer[position++];

  return {
    messageId,
    timestamp,
    activationByte,
    position,
  };
}

// 🔹 Length Descriptors Extraction
function extractLengthDescriptors(buffer, position, activeSensors) {
  const descriptors = new Map();
  let currentPos = position;

  for (const sensor of activeSensors) {
    const lengthByte = buffer[currentPos++];
    const dataLength = lengthByte & CONSTANTS.LENGTH_MASK;
    descriptors.set(sensor.bit, dataLength);
  }

  return { descriptors, newPosition: currentPos };
}

// 🔹 Generic Sensor Data Processing
function processSensorData(buffer, position, sensor, dataLength, arrivalTime) {
  const config = sensor.config;
  let rawBytes, processedValues, channelData;

  // Calculate total values and bytes needed based on config
  let totalValues, totalBytes;

  switch (config.dataSize) {
    case "1_byte":
      totalValues = config.channels * dataLength;
      totalBytes = totalValues;
      break;
      
    case "2_bytes":
      totalValues = config.channels * dataLength;
      totalBytes = totalValues * 2;
      break;
      
    case "4_bytes":
        totalValues = dataLength;
        totalBytes = totalValues * 4;
        break;
        
    case "10_bits":
      totalValues = dataLength;
      const totalBits = totalValues * CONSTANTS.CURRENT_BITS_PER_VALUE;
      totalBytes = Math.ceil(totalBits / CONSTANTS.BITS_PER_BYTE);
      break;
    default:
      throw new Error(`Unsupported data size: ${config.dataSize}`);
  }

  // Extract raw bytes
  rawBytes = buffer.slice(position, position + totalBytes);

  // Process data based on dataSize
  processedValues = processByDataSize(
    rawBytes,
    config.dataSize,
    totalValues,
    config.conversion,
    config.valueType
  );

  // Distribute to channels
  if (config.channels === 1) {
    // Single channel
    const formattedValues = formatDataWithTimestamps(processedValues, arrivalTime, config.frecuency);
    channelData = { ch1: formattedValues };
  } else {
        // Multiple channels - split values evenly
        channelData = {};

        // Inicializar los canales
        for (let ch = 0; ch < config.channels; ch++) {
            channelData[`ch${ch + 1}`] = [];
        }

        // Distribuir los valores en round-robin
        processedValues.forEach((value, index) => {
            const channel = (index % config.channels) + 1;

            channelData[`ch${channel}`].push({
                value,
                time: new Date(
                    arrivalTime +
                    Math.floor(index / config.channels) * config.frecuency
                ).toISOString(),
            });
        });
  }

  return {
    data: channelData,
    newPosition: position + totalBytes,
  };
}

// 🔹 Buffer Helper Functions
function readUInt32BE(buffer, offset) {
  return (
    (buffer[offset] << 24) |
    (buffer[offset + 1] << 16) |
    (buffer[offset + 2] << 8) |
    buffer[offset + 3]
  );
}

// 🔹 ChirpStack Export
function decodeUplink(input) {
  try {
    const hexArray = input.bytes;
    if (!hexArray || !Array.isArray(hexArray)) {
      throw new Error("Invalid input: 'bytes' array is required");
    }

    const buffer = new Uint8Array(hexArray);
    validateInput(buffer);

    // Parse header
    const header = parseHeader(buffer);
    const activeSensors = getActiveSensors(header.activationByte);
    let currentPosition = header.position;

    // Extract length descriptors
    const { descriptors, newPosition } = extractLengthDescriptors(
      buffer,
      currentPosition,
      activeSensors
    );
    currentPosition = newPosition;

    // Calculate arrival time (current timestamp)
    const arrivalTime = Date.now();

    // Initialize result
    const result = {
      data: {
        id: header.messageId,
        relative_timestamp: header.timestamp,
        arrival_date: new Date(arrivalTime).toISOString(),
        active_sensors: activeSensors.map((s) => s.name),
        measurements: {}
      }
    };

    // Process each active sensor
    for (const sensor of activeSensors) {
      const dataLength = descriptors.get(sensor.bit);
      const sensorResult = processSensorData(
        buffer,
        currentPosition,
        sensor,
        dataLength,
        arrivalTime
      );

      result.data.measurements[sensor.name] = sensorResult.data;
      currentPosition = sensorResult.newPosition;
    }

    return result;
  } catch (error) {
    return {
      error: `Decoding failed: ${error.message}`,
      raw_input: input,
    };
  }
}