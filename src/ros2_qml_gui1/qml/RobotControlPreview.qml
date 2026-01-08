import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Window 2.15

ApplicationWindow {
    id: previewWindow
    visible: true
    width: 450
    height: 900
    title: "Robot Control Panel Preview"
    color: "#0d1117"

    // Mock robot controller
    property var robotController: QtObject {
        property string systemStatus: "IDLE"
        property string errorMessage: ""
        property int selectedRow: 3
        property int selectedSlot: 5
        
        function enableSystem(enable) {
            console.log("✅ enableSystem:", enable)
            systemStatus = enable ? "RUNNING" : "STOPPED"
        }
        
        function emergencyStop(stop) {
            console.log("🛑 emergencyStop:", stop)
            if (stop) {
                systemStatus = "EMERGENCY_STOP"
                errorMessage = "Emergency stop activated!"
            } else {
                errorMessage = ""
            }
        }
        
        function setManualMode(enable) {
            console.log("🎮 setManualMode:", enable)
        }
        
        function setAiMode(enable) {
            console.log("🤖 setAiMode:", enable)
        }
        
        function switchCamera(cameraId) {
            console.log("📹 switchCamera:", cameraId)
        }
        
        function selectRow(row) {
            console.log("📥 selectRow:", row)
            robotController.selectedRow = row
        }
        
        function selectSlot(slot) {
            console.log("📤 selectSlot:", slot)
            robotController.selectedSlot = slot
        }
        
        function gotoState(state) {
            console.log("🔄 gotoState:", state)
        }
    }

    Rectangle {
        anchors.fill: parent
        color: "#081e29"
        border.color: "#134357"
        radius: 6

        ScrollView {
            anchors.fill: parent
            anchors.margins: 10
            clip: true
            
            GridLayout {
                width: parent.width
                columns: 1
                rowSpacing: 15

                // Robot Control Section
                Text {
                    text: "🤖 ROBOT CONTROL"
                    color: "#5cf4f1"
                    font.bold: true
                    font.pixelSize: 18
                    Layout.alignment: Qt.AlignHCenter
                }

                Rectangle {
                    height: 1
                    Layout.fillWidth: true
                    color: "#134357"
                }

                // System Status
                GridLayout {
                    Layout.fillWidth: true
                    columns: 2
                    rowSpacing: 10
                    columnSpacing: 10
                    
                    Text {
                        text: "Status:"
                        color: "#94a3b8"
                        font.pixelSize: 14
                    }
                    Text {
                        text: robotController.systemStatus
                        color: "#10b981"
                        font.pixelSize: 14
                        font.bold: true
                    }
                    
                    Text {
                        text: "Row:"
                        color: "#94a3b8"
                        font.pixelSize: 14
                    }
                    Text {
                        text: robotController.selectedRow > 0 ? robotController.selectedRow : "-"
                        color: "#6366f1"
                        font.pixelSize: 14
                        font.bold: true
                    }
                    
                    Text {
                        text: "Slot:"
                        color: "#94a3b8"
                        font.pixelSize: 14
                    }
                    Text {
                        text: robotController.selectedSlot > 0 ? robotController.selectedSlot : "-"
                        color: "#6366f1"
                        font.pixelSize: 14
                        font.bold: true
                    }
                }

                // Error Display
                Rectangle {
                    Layout.fillWidth: true
                    height: errorText.visible ? 60 : 0
                    color: "#ef444420"
                    border.color: "#ef4444"
                    border.width: 1
                    radius: 6
                    visible: robotController.errorMessage !== ""
                    
                    Text {
                        id: errorText
                        anchors.fill: parent
                        anchors.margins: 8
                        text: robotController.errorMessage
                        color: "#ef4444"
                        font.pixelSize: 12
                        wrapMode: Text.WordWrap
                        visible: robotController.errorMessage !== ""
                    }
                }

                Rectangle {
                    height: 1
                    Layout.fillWidth: true
                    color: "#134357"
                }

                // System Control Buttons
                Text {
                    text: "System Control"
                    color: "#94a3b8"
                    font.pixelSize: 14
                    font.bold: true
                }

                Button {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 45
                    text: "▶️ Enable System"
                    onClicked: robotController.enableSystem(true)
                    
                    background: Rectangle {
                        radius: 8
                        color: parent.pressed ? "#059669" : "#10b981"
                        border.color: "#10b981"
                        border.width: 2
                    }
                    
                    contentItem: Text {
                        text: parent.text
                        color: "white"
                        font.pixelSize: 14
                        font.bold: true
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                }

                Button {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 45
                    text: "⏸️ Disable System"
                    onClicked: robotController.enableSystem(false)
                    
                    background: Rectangle {
                        radius: 8
                        color: parent.pressed ? "#475569" : "#64748b"
                        border.color: "#64748b"
                        border.width: 2
                    }
                    
                    contentItem: Text {
                        text: parent.text
                        color: "white"
                        font.pixelSize: 14
                        font.bold: true
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                }

                Button {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 45
                    text: "🛑 Emergency Stop"
                    onClicked: robotController.emergencyStop(true)
                    
                    background: Rectangle {
                        radius: 8
                        color: parent.pressed ? "#dc2626" : "#ef4444"
                        border.color: "#ef4444"
                        border.width: 2
                    }
                    
                    contentItem: Text {
                        text: parent.text
                        color: "white"
                        font.pixelSize: 14
                        font.bold: true
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                }

                Rectangle {
                    height: 1
                    Layout.fillWidth: true
                    color: "#134357"
                }

                // Mode Control
                Text {
                    text: "Operation Mode"
                    color: "#94a3b8"
                    font.pixelSize: 14
                    font.bold: true
                }

                RowLayout {
                    Layout.fillWidth: true
                    spacing: 8
                    
                    Button {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 40
                        text: "🎮 Manual"
                        onClicked: {
                            robotController.setManualMode(true)
                            robotController.setAiMode(false)
                        }
                        
                        background: Rectangle {
                            radius: 8
                            color: parent.pressed ? "#4f46e5" : "#6366f1"
                            border.color: "#6366f1"
                            border.width: 2
                        }
                        
                        contentItem: Text {
                            text: parent.text
                            color: "white"
                            font.pixelSize: 13
                            font.bold: true
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                        }
                    }
                    
                    Button {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 40
                        text: "🤖 AI"
                        onClicked: {
                            robotController.setManualMode(false)
                            robotController.setAiMode(true)
                        }
                        
                        background: Rectangle {
                            radius: 8
                            color: parent.pressed ? "#7c3aed" : "#8b5cf6"
                            border.color: "#8b5cf6"
                            border.width: 2
                        }
                        
                        contentItem: Text {
                            text: parent.text
                            color: "white"
                            font.pixelSize: 13
                            font.bold: true
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                        }
                    }
                }

                Rectangle {
                    height: 1
                    Layout.fillWidth: true
                    color: "#134357"
                }

                // Camera Control
                Text {
                    text: "Camera Control"
                    color: "#94a3b8"
                    font.pixelSize: 14
                    font.bold: true
                }

                RowLayout {
                    Layout.fillWidth: true
                    spacing: 8
                    
                    Button {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 40
                        text: "📹 Cam 0"
                        onClicked: robotController.switchCamera(0)
                        
                        background: Rectangle {
                            radius: 8
                            color: parent.pressed ? "#0891b2" : "#06b6d4"
                            border.color: "#06b6d4"
                            border.width: 2
                        }
                        
                        contentItem: Text {
                            text: parent.text
                            color: "white"
                            font.pixelSize: 13
                            font.bold: true
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                        }
                    }
                    
                    Button {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 40
                        text: "📹 Cam 1"
                        onClicked: robotController.switchCamera(1)
                        
                        background: Rectangle {
                            radius: 8
                            color: parent.pressed ? "#0891b2" : "#06b6d4"
                            border.color: "#06b6d4"
                            border.width: 2
                        }
                        
                        contentItem: Text {
                            text: parent.text
                            color: "white"
                            font.pixelSize: 13
                            font.bold: true
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                        }
                    }
                }

                Rectangle {
                    height: 1
                    Layout.fillWidth: true
                    color: "#134357"
                }

                // Row Selection
                Text {
                    text: "Select Input Row"
                    color: "#94a3b8"
                    font.pixelSize: 14
                    font.bold: true
                }

                GridLayout {
                    Layout.fillWidth: true
                    columns: 5
                    columnSpacing: 4
                    rowSpacing: 4
                    
                    Repeater {
                        model: 5
                        Button {
                            Layout.fillWidth: true
                            Layout.preferredHeight: 35
                            text: (index + 1).toString()
                            onClicked: robotController.selectRow(index + 1)
                            
                            background: Rectangle {
                                radius: 6
                                color: parent.pressed ? "#f59e0b" : "#fbbf24"
                                border.color: "#f59e0b"
                                border.width: 2
                            }
                            
                            contentItem: Text {
                                text: parent.text
                                color: "#0d1117"
                                font.pixelSize: 14
                                font.bold: true
                                horizontalAlignment: Text.AlignHCenter
                                verticalAlignment: Text.AlignVCenter
                            }
                        }
                    }
                }

                Rectangle {
                    height: 1
                    Layout.fillWidth: true
                    color: "#134357"
                }

                // Slot Selection
                Text {
                    text: "Select Output Slot"
                    color: "#94a3b8"
                    font.pixelSize: 14
                    font.bold: true
                }

                GridLayout {
                    Layout.fillWidth: true
                    columns: 4
                    columnSpacing: 4
                    rowSpacing: 4
                    
                    Repeater {
                        model: 8
                        Button {
                            Layout.fillWidth: true
                            Layout.preferredHeight: 35
                            text: (index + 1).toString()
                            onClicked: robotController.selectSlot(index + 1)
                            
                            background: Rectangle {
                                radius: 6
                                color: parent.pressed ? "#14b8a6" : "#2dd4bf"
                                border.color: "#14b8a6"
                                border.width: 2
                            }
                            
                            contentItem: Text {
                                text: parent.text
                                color: "#0d1117"
                                font.pixelSize: 14
                                font.bold: true
                                horizontalAlignment: Text.AlignHCenter
                                verticalAlignment: Text.AlignVCenter
                            }
                        }
                    }
                }

                Rectangle {
                    height: 1
                    Layout.fillWidth: true
                    color: "#134357"
                }

                // Info
                Text {
                    text: "Preview Mode - Check console for button clicks"
                    color: "#94a3b8"
                    font.pixelSize: 11
                    font.italic: true
                    Layout.alignment: Qt.AlignHCenter
                    wrapMode: Text.WordWrap
                    Layout.fillWidth: true
                    horizontalAlignment: Text.AlignHCenter
                }
            }
        }
    }
}
