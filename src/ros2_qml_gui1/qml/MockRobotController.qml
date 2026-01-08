import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Window 2.15

// Mock RobotController for preview
QtObject {
    id: robotController
    property string systemStatus: "IDLE"
    property string errorMessage: ""
    property int selectedRow: 3
    property int selectedSlot: 5
    
    function enableSystem(enable) {
        console.log("enableSystem:", enable)
        systemStatus = enable ? "RUNNING" : "STOPPED"
    }
    
    function emergencyStop(stop) {
        console.log("emergencyStop:", stop)
        if (stop) {
            systemStatus = "EMERGENCY_STOP"
            errorMessage = "Emergency stop activated!"
        } else {
            errorMessage = ""
        }
    }
    
    function setManualMode(enable) {
        console.log("setManualMode:", enable)
    }
    
    function setAiMode(enable) {
        console.log("setAiMode:", enable)
    }
    
    function switchCamera(cameraId) {
        console.log("switchCamera:", cameraId)
    }
    
    function selectRow(row) {
        console.log("selectRow:", row)
        selectedRow = row
    }
    
    function selectSlot(slot) {
        console.log("selectSlot:", slot)
        selectedSlot = slot
    }
    
    function gotoState(state) {
        console.log("gotoState:", state)
    }
}
