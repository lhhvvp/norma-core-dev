import React, { useState, useMemo } from 'react';
import webSocketManager from '../api/websocket';
import { useInferenceState, useWakeLock } from '../hooks';
import { st3215 } from '../api/proto';
import { useLocation, Link } from 'react-router-dom';

const St3215MotorConfigPage: React.FC = () => {
  useWakeLock();
  const location = useLocation();
  const selectedBusFromState = location.state?.bus as st3215.InferenceState.IBusState | undefined;

  const inferenceState = useInferenceState();
  const [isMotorIdSetInProgress, setIsMotorIdSetInProgress] = useState(false);
  const [newMotorId, setNewMotorId] = useState<number>(1);

  // Derive selected bus from inference state, falling back to router state
  const selectedBus = useMemo(() => {
    if (!selectedBusFromState?.bus?.serialNumber) return null;
    
    // Try to get updated bus from inference state
    const updatedBus = inferenceState?.st3215?.data.buses?.find(
      (bus: st3215.InferenceState.IBusState) => bus.bus?.serialNumber === selectedBusFromState.bus?.serialNumber
    );

    return updatedBus || selectedBusFromState;
  }, [inferenceState?.st3215?.data.buses, selectedBusFromState]);


  const getMotorIdFromState = (data: Uint8Array): number => {
    // Motor ID is typically at address 0x05 in ST3215
    if (data.length > 0x05) {
      return data[0x05];
    }
    return 0;
  };

  // Send a command without waiting for response
  const sendCommand = async (busSerial: string, command: st3215.ICommand): Promise<void> => {
    await webSocketManager.commands.sendSt3215Command({
        targetBusSerial: busSerial,
        ...command
      });
  };


  const handleSetMotorId = () => {
    if (!selectedBus?.bus?.serialNumber) {
      console.error('No bus selected for motor ID setting');
      return;
    }

    if (isMotorIdSetInProgress) {
      console.log('Motor ID setting already in progress');
      return;
    }

    if (newMotorId < 1 || newMotorId > 10) {
      console.error('Motor ID must be between 1 and 10');
      return;
    }

    const busSerial = selectedBus.bus.serialNumber;
    const currentMotorId = selectedBus.motors?.[0]?.id || 1;

    setIsMotorIdSetInProgress(true);

    try {
      console.log(`Starting motor ID setting from ${currentMotorId} to ${newMotorId} on bus ${busSerial}`);

      // Send all commands sequentially without waiting
      sendCommand(busSerial, {
        write: {
          motorId: currentMotorId,
          address: 0x37, // Lock register
          value: new Uint8Array([0]) // Unlock
        }
      });

      sendCommand(busSerial, {
        action: {
          motorId: currentMotorId
        }
      });

      sendCommand(busSerial, {
        write: {
          motorId: currentMotorId,
          address: 0x05, // Motor ID register
          value: new Uint8Array([newMotorId])
        }
      });

      sendCommand(busSerial, {
        action: {
          motorId: newMotorId // Use new motor ID
        }
      });

      sendCommand(busSerial, {
        write: {
          motorId: newMotorId, // Use new motor ID
          address: 0x37, // Lock register
          value: new Uint8Array([1]) // Lock
        }
      });

      sendCommand(busSerial, {
        action: {
          motorId: newMotorId // Use new motor ID
        }
      });

      console.log(`Motor ID setting commands sent: ${currentMotorId} -> ${newMotorId}`);

    } catch (error) {
      console.error('Motor ID setting failed:', error);
    } finally {
      setIsMotorIdSetInProgress(false);
    }
  };

  if (selectedBus) {
    const motor = selectedBus.motors?.[0];
    const motorState = motor?.state;
    const motorId = motorState ? getMotorIdFromState(motorState) : 0;

    return (
      <div className="min-h-screen bg-black text-green-400 font-mono p-6">
        <div className="container mx-auto">
          <div className="flex items-center gap-4 mb-6">
            <Link
              to="/"
              className="px-4 py-2 bg-gray-600 text-white rounded hover:bg-gray-500 transition-colors"
            >
              ← Back to Home
            </Link>
            <h1 className="text-3xl font-bold text-cyan-400">
              Bus: {selectedBus.bus?.serialNumber || 'Unknown'}
            </h1>
          </div>

          {motor && motorState ? (
            <div className="space-y-6">
              {/* Motor Info */}
              <div className="bg-gray-900 rounded-lg p-6">
                <h2 className="text-xl font-bold text-yellow-400 mb-4">Motor Information</h2>
                <div className="grid grid-cols-2 md:grid-cols-4 gap-4 text-sm mb-4">
                  <div>
                    <span className="text-gray-500">Current Motor ID:</span>
                    <span className="text-green-400 ml-2 font-bold">{motorId}</span>
                  </div>
                  <div>
                    <span className="text-gray-500">State Size:</span>
                    <span className="text-cyan-400 ml-2">{motorState.length} bytes</span>
                  </div>
                </div>
              </div>

              {/* Motor State Hex Dump */}
              <div className="bg-gray-900 rounded-lg p-6">
                <h2 className="text-xl font-bold text-yellow-400 mb-4">Motor State (Hex Dump)</h2>
                <div className="bg-black rounded-lg p-4 font-mono text-sm overflow-x-auto">
                  <div className="text-gray-500 mb-2">Address  00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F</div>
                  {Array.from({ length: Math.ceil(motorState.length / 16) }, (_, rowIndex) => {
                    const startAddr = rowIndex * 16;
                    const rowData = motorState.slice(startAddr, startAddr + 16);
                    return (
                      <div key={rowIndex} className="flex gap-2">
                        <span className="text-gray-500 w-16">
                          {startAddr.toString(16).padStart(8, '0').toUpperCase()}
                        </span>
                        <span className="text-green-400">
                          {Array.from(rowData)
                            .map(byte => byte.toString(16).padStart(2, '0').toUpperCase())
                            .join(' ')
                            .padEnd(47, ' ')}
                        </span>
                        <span className="text-gray-400">
                          {Array.from(rowData)
                            .map(byte => (byte >= 32 && byte <= 126) ? String.fromCharCode(byte) : '.')
                            .join('')}
                        </span>
                      </div>
                    );
                  })}
                </div>
              </div>

              {/* Control Buttons */}
              <div className="bg-gray-900 rounded-lg p-6">
                <h2 className="text-xl font-bold text-yellow-400 mb-4">Motor Control</h2>
                
                {/* Motor ID Setting */}
                <div className="mb-6">
                  <h3 className="text-lg font-bold text-cyan-400 mb-3">Set Motor ID</h3>
                  <div className="flex items-center gap-4">
                    <div className="flex items-center gap-2">
                      <label htmlFor="motorId" className="text-gray-400 text-sm">New Motor ID:</label>
                      <input
                        id="motorId"
                        type="number"
                        min="1"
                        max="10"
                        value={newMotorId}
                        onClick={(e) => (e.target as HTMLInputElement).select()}
                        onChange={(e) => setNewMotorId(Math.max(1, Math.min(10, parseInt(e.target.value) || 1)))}
                        disabled={isMotorIdSetInProgress}
                        className="w-20 px-3 py-2 bg-gray-800 text-green-400 border border-gray-600 rounded focus:border-cyan-400 focus:outline-none disabled:opacity-50"
                      />
                      <span className="text-gray-500 text-sm">(1-10)</span>
                    </div>
                    <button
                      onClick={handleSetMotorId}
                      disabled={isMotorIdSetInProgress || newMotorId < 1 || newMotorId > 10}
                      className={`px-6 py-2 rounded-lg transition-colors font-bold ${
                        isMotorIdSetInProgress || newMotorId < 1 || newMotorId > 10
                          ? 'bg-gray-600 text-gray-400 cursor-not-allowed'
                          : 'bg-blue-600 text-white hover:bg-blue-700'
                      }`}
                    >
                      {isMotorIdSetInProgress ? 'Setting Motor ID...' : 'Set Motor ID'}
                    </button>
                  </div>
                </div>

              </div>

            </div>
          ) : (
            <div className="bg-gray-900 rounded-lg p-6">
              <div className="text-gray-400">No motor data available for this bus.</div>
            </div>
          )}
        </div>
      </div>
    );
  }

  return (
    <div className="min-h-screen bg-black text-green-400 font-mono p-6">
      <div className="container mx-auto">
        <h1 className="text-3xl font-bold text-cyan-400 mb-4">ST3215 Motor ID Configuration</h1>
        <p className="text-gray-400 mb-4">
          No bus selected for configuration. Please go back to the main page and select a bus.
        </p>
        <Link
            to="/"
            className="px-4 py-2 bg-gray-600 text-white rounded hover:bg-gray-500 transition-colors"
        >
            ← Back to Home
        </Link>
      </div>
    </div>
  );
};

export default St3215MotorConfigPage;
