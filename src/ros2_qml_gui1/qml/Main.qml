import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Controls.Material 2.15
import QtQuick.Window 2.15
import QtCharts 2.15

ApplicationWindow {
    id: mainWindow 
    visible: true
    width: 1920
    height: 1080
    title: "ROS2 - CAMERA VIEWER"
    color: "#0d1117"
    visibility: Window.FullScreen
    property string currentTime: Qt.formatDateTime(new Date(), "yyyy-MM-dd hh:mm")

    Shortcut {
        sequence: "F11"
        onActivated: {
            if (mainWindow.visibility === Window.FullScreen)
                mainWindow.visibility = Window.Windowed
            else
                mainWindow.visibility = Window.FullScreen
        }
    }

    ColumnLayout {
        anchors.fill: parent
        spacing: 0

        // Title row
        Item {
            id: itemTitleRow
            width: parent.width
            height: 80

            property string currentTime: Qt.formatDateTime(new Date(), "yyyy-MM-dd hh:mm:ss")

            Timer {
                interval: 1000
                running: true
                repeat: true
                onTriggered: {
                    currentTime = Qt.formatDateTime(new Date(), "yyyy-MM-dd hh:mm:ss")
                }
            }

            Rectangle {
                anchors.fill: parent
                color: "transparent"

                RowLayout {
                    anchors.fill: parent
                    anchors.margins: 10
                    spacing: 10
                    Item {
                        Layout.preferredWidth: 60
                    }
                    

                    
                    Item {
                        Layout.fillWidth: true

                        Text {
                            id: titleText
                            anchors.centerIn: parent
                            text: "ROS2 - ROBOT CONTROL SYSTEM"
                            font.pixelSize: 24
                            font.bold: true
                            color: "#6cf"
                        }
                    }

                    Button {
                        Layout.preferredWidth: 50
                        Layout.preferredHeight: 50

                        onClicked: {
                            var comp = Qt.createComponent("frm_settings.qml");
                            if (comp.status === Component.Ready) {
                                var win = comp.createObject(mainWindow);
                                if (win) {
                                    win.x = mainWindow.x + (mainWindow.width - win.width) / 2;
                                    win.y = mainWindow.y + (mainWindow.height - win.height) / 2;
                                    win.show();
                                } else {
                                    console.log("createObject failed");
                                }
                            } else {
                                console.log("Failed to load settings:", comp.errorString());
                            }
                        }

                        background: Rectangle {
                            radius: 6
                            color: "transparent"
                            border.color: "#134357"
                            border.width: 2
                        }
                        
                        contentItem: Image {
                            source: "qrc:/icons/qml/icons/settings.svg"
                            width: 24
                            height: 24
                            fillMode: Image.PreserveAspectFit
                            smooth: true
                        }
                    }
                    
                    Button {
                        Layout.preferredWidth: 50
                        Layout.preferredHeight: 50
                        onClicked: Qt.quit()

                        background: Rectangle {
                            radius: 6
                            color: "transparent"
                            border.color: "#134357"
                            border.width: 2
                        }

                        contentItem: Image {
                            source: "qrc:/icons/qml/icons/power_settings.svg"
                            width: 24
                            height: 24
                            fillMode: Image.PreserveAspectFit
                            smooth: true
                        }
                    }
                }
            }
        }


        // Main content
        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 10
            
            // Middle Left: Camera Area (Flexible height, but prioritized)
            Rectangle{
                color: "#081e29"
                Layout.fillHeight: true
                Layout.fillWidth: true
                Layout.preferredWidth: 3.5
                border.color: "#134357"
                radius: 6


                GridLayout {
                    id: camGrid
                    columns: 2
                    rowSpacing: 10
                    columnSpacing: 10
                    anchors.fill: parent
                    anchors.margins: 10
                    
                    Repeater {
                        model: camNode.cameraList

                        delegate: CameraView {
                            cameraName: modelData.name
                            topic: modelData.topic
                            providerId: modelData.providerId
                            
                            // Let GridLayout handle sizing, but hint aspect ratio could be enforced inside CameraView
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                        }
                    }
                }
            }

            // Middle Right: Status Panel (Vertical)
            Rectangle {
                Layout.fillHeight: true
                Layout.preferredWidth: 350
                Layout.minimumWidth: 300
                color: "#081e29"
                border.color: "#134357"
                radius: 6

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 10
                    spacing: 15

                    Text {
                        text: "SYSTEM MONITOR"
                        color: "#5cf4f1"
                        font.bold: true
                        font.pixelSize: 16
                        Layout.alignment: Qt.AlignHCenter
                    }
                    
                    Rectangle { Layout.fillWidth: true; height: 1; color: "#134357" }

                    // Status Grid
                    GridLayout {
                        Layout.fillWidth: true
                        columns: 2
                        rowSpacing: 10
                        columnSpacing: 10
                        
                        Text { text: "Status:"; color: "#94a3b8"; font.pixelSize: 13 }
                        Text { text: robotController.systemStatus; color: "#10b981"; font.bold: true; font.pixelSize: 13 }
                        
                        Text { text: "Mode:"; color: "#94a3b8"; font.pixelSize: 13 }
                        Text { 
                            text: "MANUAL" // Placeholder, should bind to actual mode if available
                            color: "#6366f1"; font.bold: true; font.pixelSize: 13 
                        }

                        Text { text: "Row:"; color: "#94a3b8"; font.pixelSize: 13 }
                        Text { text: robotController.selectedRow > 0 ? robotController.selectedRow : "-"; color: "#5cf4f1"; font.bold: true; font.pixelSize: 13 }

                        Text { text: "Slot:"; color: "#94a3b8"; font.pixelSize: 13 }
                        Text { text: robotController.selectedSlot > 0 ? robotController.selectedSlot : "-"; color: "#5cf4f1"; font.bold: true; font.pixelSize: 13 }
                    }

                    Rectangle { Layout.fillWidth: true; height: 1; color: "#134357" }
                    
                    // Error Display
                    Rectangle {
                        Layout.fillWidth: true
                        height: errorText.visible ? 60 : 0
                        color: "#ef444420"
                        border.color: "#ef4444"
                        radius: 6
                        visible: robotController.errorMessage !== ""
                        
                        Text {
                            id: errorText
                            anchors.fill: parent
                            anchors.margins: 5
                            text: robotController.errorMessage
                            color: "#ef4444"
                            font.pixelSize: 12
                            wrapMode: Text.WordWrap
                        }
                    }
                       // Main System Controls
                    Text { text: "System Control"; color: "#5cf4f1"; font.bold: true; font.pixelSize: 14 }
                    Button {
                        Layout.fillWidth: true; Layout.preferredHeight: 50
                        text: "▶️ Enable System"
                        onClicked: robotController.enableSystem(true)
                        background: Rectangle { radius: 6; color: parent.pressed ? "#5cf4f1" : "transparent"; border.color: "#5cf4f1"; border.width: 2 }
                        contentItem: Text { text: parent.text; color: parent.pressed ? "#0d1117" : "#5cf4f1"; font.bold: true; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                    }
                    Button {
                        Layout.fillWidth: true; Layout.preferredHeight: 50
                        text: "II Pause System"
                        onClicked: robotController.enableSystem(false)
                        background: Rectangle { radius: 6; color: parent.pressed ? "#5cf4f1" : "transparent"; border.color: "#5cf4f1"; border.width: 2 }
                        contentItem: Text { text: parent.text; color: parent.pressed ? "#0d1117" : "#5cf4f1"; font.bold: true; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                    }
                    Button {
                        Layout.fillWidth: true; Layout.preferredHeight: 55
                        text: "STOP EMERGENCY"
                        onClicked: robotController.emergencyStop(true)
                        background: Rectangle { radius: 6; color: parent.pressed ? "#dc2626" : "#ef4444"; border.color: "#ef4444"; border.width: 2 }
                        contentItem: Text { text: parent.text; color: "white"; font.bold: true; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                    }
                    // Mode Controls
                    Text { text: "Operation Mode"; color: "#5cf4f1"; font.bold: true; font.pixelSize: 14 }
                        Button {
                            Layout.fillWidth: true; Layout.preferredHeight: 50; text: "  Auto"
                            onClicked: { 
                                robotController.setManualMode(false); 
                                robotController.setAiMode(false); 
                            }
                            background: Rectangle { radius: 8; color: parent.pressed ? "#5cf4f1" : "transparent"; border.color: "#5cf4f1"; border.width: 2 }
                            contentItem: Text { text: parent.text; color: parent.pressed ? "#0d1117" : "#5cf4f1"; font.bold: true; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                        }
                    Button {
                        Layout.fillWidth: true; Layout.preferredHeight: 50; text: "  AI"
                        onClicked: { 
                            robotController.setManualMode(false); 
                            robotController.setAiMode(true); 
                        }
                        background: Rectangle { radius: 8; color: parent.pressed ? "#5cf4f1" : "transparent"; border.color: "#5cf4f1"; border.width: 2 }
                        contentItem: Text { text: parent.text; color: parent.pressed ? "#0d1117" : "#5cf4f1"; font.bold: true; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                    }
                    Button {
                        Layout.fillWidth: true; Layout.preferredHeight: 50; text: "  Manual"
                        onClicked: { 
                            robotController.setManualMode(true); 
                            robotController.setAiMode(false); 
                        }
                        background: Rectangle { radius: 8; color: parent.pressed ? "#5cf4f1" : "transparent"; border.color: "#5cf4f1"; border.width: 2 }
                        contentItem: Text { text: parent.text; color: parent.pressed ? "#0d1117" : "#5cf4f1"; font.bold: true; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                    }

                    // Camera Controls
                    Text { text: "Camera Control"; color: "#5cf4f1"; font.bold: true; font.pixelSize: 14 }
                    RowLayout {
                        Layout.fillWidth: true
                        Button {
                            Layout.fillWidth: true; Layout.preferredHeight: 50; text: " Cam 0"
                            onClicked: robotController.switchCamera(0)
                            background: Rectangle { radius: 6; color: parent.pressed ? "#5cf4f1" : "transparent"; border.color: "#5cf4f1"; border.width: 2 }
                            contentItem: Text { text: parent.text; color: parent.pressed ? "#0d1117" : "#5cf4f1"; font.bold: true; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                        }
                        Button {
                            Layout.fillWidth: true; Layout.preferredHeight: 50; text: " Cam 1"
                            onClicked: robotController.switchCamera(1)
                            background: Rectangle { radius: 6; color: parent.pressed ? "#5cf4f1" : "transparent"; border.color: "#5cf4f1"; border.width: 2 }
                            contentItem: Text { text: parent.text; color: parent.pressed ? "#0d1117" : "#5cf4f1"; font.bold: true; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                        }
                    }

                    Item { Layout.fillHeight: true } // Spacer
                }
            }
        }

        // Bottom Bar: Control Sequence (Horizontal)
        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 160
            color: "#081e29"
            border.color: "#134357"
            radius: 6
            
            RowLayout {
                anchors.fill: parent
                anchors.margins: 10
                spacing: 20

                // 1. Select Input Row
                Rectangle {
                    Layout.fillHeight: true
                    Layout.preferredWidth: parent.width * 0.2
                    Layout.fillWidth: true
                    Layout.minimumWidth: 180
                    color: "transparent"
                    border.color: "#134357"
                    radius: 4
                    
                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 5
                        Text { text: "1. Select Row"; color: "#5cf4f1"; font.bold: true; Layout.alignment: Qt.AlignHCenter }
                        
                        GridLayout {
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            columns: 3
                            Repeater {
                                model: 5
                                Button {
                                    Layout.fillWidth: true; Layout.fillHeight: true
                                    text: (index + 1).toString()
                                    onClicked: robotController.selectRow(index + 1)
                                    background: Rectangle {
                                        radius: 4
                                        color: parent.pressed || robotController.selectedRow === (index + 1) ? "#5cf4f1" : "transparent"
                                        border.color: "#5cf4f1"
                                        border.width: 1
                                    }
                                    contentItem: Text {
                                        text: parent.text
                                        color: parent.pressed || robotController.selectedRow === (index + 1) ? "#0d1117" : "#5cf4f1"
                                        font.bold: true
                                        horizontalAlignment: Text.AlignHCenter
                                        verticalAlignment: Text.AlignVCenter
                                    }
                                }
                            }
                        }
                    }
                }

                // 2. Select Output Slot
                Rectangle {
                    Layout.fillHeight: true
                    Layout.preferredWidth: parent.width * 0.3
                    Layout.fillWidth: true
                    Layout.minimumWidth: 250
                    color: "transparent"
                    border.color: "#134357"
                    radius: 4
                    
                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 5
                        Text { text: "2. Select Slot"; color: "#5cf4f1"; font.bold: true; Layout.alignment: Qt.AlignHCenter }
                        
                        GridLayout {
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            columns: 4
                            Repeater {
                                model: 8
                                Button {
                                    Layout.fillWidth: true; Layout.fillHeight: true
                                    text: (index + 1).toString()
                                    onClicked: robotController.selectSlot(index + 1)
                                    background: Rectangle {
                                        radius: 4
                                        color: parent.pressed || robotController.selectedSlot === (index + 1) ? "#5cf4f1" : "transparent"
                                        border.color: "#5cf4f1"
                                        border.width: 1
                                    }
                                    contentItem: Text {
                                        text: parent.text
                                        color: parent.pressed || robotController.selectedSlot === (index + 1) ? "#0d1117" : "#5cf4f1"
                                        font.bold: true
                                        horizontalAlignment: Text.AlignHCenter
                                        verticalAlignment: Text.AlignVCenter
                                    }
                                }
                            }
                        }
                    }
                }

                // 3. Robot Actions
                Rectangle {
                    Layout.fillHeight: true
                    Layout.preferredWidth: parent.width * 0.45
                    Layout.fillWidth: true
                    color: "transparent"
                    border.color: "#134357"
                    radius: 4
                    
                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 5
                        Text { text: "3. Execute Action"; color: "#5cf4f1"; font.bold: true; Layout.alignment: Qt.AlignHCenter }
                        
                        GridLayout {
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            columns: 3 // Set to 3 columns for 3x2 layout
                            
                            Button {
                                Layout.fillWidth: true; Layout.fillHeight: true; text: "Pick Direct"
                                onClicked: robotController.gotoState("INIT_LOAD_CHAMBER_DIRECT")
                                background: Rectangle { radius: 4; color: parent.pressed ? "#5cf4f1" : "transparent"; border.color: "#5cf4f1"; border.width: 1 }
                                contentItem: Text { text: parent.text; color: parent.pressed ? "#0d1117" : "#5cf4f1"; font.bold: true; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                            }
                            Button {
                                Layout.fillWidth: true; Layout.fillHeight: true; text: "Refill Buffer"
                                onClicked: robotController.gotoState("REFILL_BUFFER")
                                background: Rectangle { radius: 4; color: parent.pressed ? "#5cf4f1" : "transparent"; border.color: "#5cf4f1"; border.width: 1 }
                                contentItem: Text { text: parent.text; color: parent.pressed ? "#0d1117" : "#5cf4f1"; font.bold: true; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                            }
                            Button {
                                Layout.fillWidth: true; Layout.fillHeight: true; text: "Put To Chamber"
                                onClicked: robotController.gotoState("LOAD_CHAMBER_FROM_BUFFER")
                                background: Rectangle { radius: 4; color: parent.pressed ? "#5cf4f1" : "transparent"; border.color: "#5cf4f1"; border.width: 1 }
                                contentItem: Text { text: parent.text; color: parent.pressed ? "#0d1117" : "#5cf4f1"; font.bold: true; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                            }
                            Button {
                                Layout.fillWidth: true; Layout.fillHeight: true; text: "Place To Scale"
                                onClicked: robotController.gotoState("TAKE_CHAMBER_TO_SCALE")
                                background: Rectangle { radius: 4; color: parent.pressed ? "#5cf4f1" : "transparent"; border.color: "#5cf4f1"; border.width: 1 }
                                contentItem: Text { text: parent.text; color: parent.pressed ? "#0d1117" : "#5cf4f1"; font.bold: true; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                            }
                            Button {
                                Layout.fillWidth: true; Layout.fillHeight: true; text: "Place To Output"
                                onClicked: robotController.gotoState("PLACE_TO_OUTPUT")
                                background: Rectangle { radius: 4; color: parent.pressed ? "#5cf4f1" : "transparent"; border.color: "#5cf4f1"; border.width: 1 }
                                contentItem: Text { text: parent.text; color: parent.pressed ? "#0d1117" : "#5cf4f1"; font.bold: true; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                            }
                            Button {
                                Layout.fillWidth: true; Layout.fillHeight: true; text: "Place To Fail"
                                onClicked: robotController.gotoState("PLACE_TO_FAIL")
                                background: Rectangle { radius: 4; color: parent.pressed ? "#5cf4f1" : "transparent"; border.color: "#5cf4f1"; border.width: 1 }
                                contentItem: Text { text: parent.text; color: parent.pressed ? "#0d1117" : "#5cf4f1"; font.bold: true; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                            }
                        }
                    }
                }
            }
        }

        Item {
            height: 40
            Layout.fillWidth: true

            Rectangle {
                anchors.fill: parent
                color: "#0d2538"
                border.color: "#134357"

                RowLayout {
                    anchors.fill: parent
                    anchors.margins: 10
                    Text {
                        text: "© 2025 RYNAN TECHNOLOGIES"
                        color: "#6cf"
                        font.pixelSize: 16
                        Layout.alignment: Qt.AlignVCenter
                    }

                    Item { Layout.fillWidth: true }

                    RowLayout {
                        spacing: 6

                        Image {
                            source: "qrc:/icons/qml/icons/app_badging.svg"
                            width: 24
                            height: 24
                            fillMode: Image.PreserveAspectFit
                            smooth: true
                            Layout.preferredWidth: 24
                            Layout.preferredHeight: 24
                            Layout.alignment: Qt.AlignVCenter
                        }

                        Text {
                            text: "Status: Running"
                            color: "#00ff99"
                            font.pixelSize: 16
                            Layout.alignment: Qt.AlignVCenter
                        }
                    }

                    Rectangle{
                        width: 2
                        Layout.fillHeight: true
                        color:"#134357"
                    }

                    RowLayout {
                        spacing: 6
                        Layout.alignment: Qt.AlignVCenter

                        Image {
                            source: "qrc:/icons/qml/icons/schedule.svg"
                            fillMode: Image.PreserveAspectFit
                            smooth: true
                            Layout.preferredWidth: 24
                            Layout.preferredHeight: 24
                            Layout.alignment: Qt.AlignVCenter
                        }

                        Text {
                            text: currentTime
                            font.pixelSize: 16
                            color: "#6cf"
                            Layout.alignment: Qt.AlignVCenter
                        }
                    }
                }
            }
        }
    }
}
