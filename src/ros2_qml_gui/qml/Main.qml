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
            height: 110

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
                    
                    Item{
                        Layout.preferredWidth: 60
                        Layout.leftMargin: 10
                        Image{
                            source: "qrc:/icons/qml/icons/rynan_logo.png"
                            anchors.centerIn: parent
                            height: 60
                            fillMode: Image.PreserveAspectFit
                            smooth: true
                        }
                    }
                    
                    Item {
                        Layout.fillWidth: true

                        Text {
                            id: titleText
                            anchors.centerIn: parent
                            text: "ROS2 - CAMERA VIEWER"
                            font.pixelSize: 28
                            font.bold: true
                            color: "#6cf"
                        }
                    }

                    Button {
                        Layout.preferredWidth: 60
                        Layout.preferredHeight: 60

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
                        Layout.preferredWidth: 60
                        Layout.preferredHeight: 60
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

            Rectangle{
                color: "#081e29"
                width: (parent.width * 0.6) - 30
                height: parent.height
                border.color: "#134357"
                radius: 6

                anchors.left: parent.left
                anchors.leftMargin: 10
                anchors.bottom: parent.bottom
                anchors.bottomMargin:10

                GridLayout {
                    id: camGrid
                    columns: 3
                    rowSpacing: 30
                    columnSpacing: 15
                    anchors.fill: parent
                    anchors.margins: 15
                    anchors.topMargin:20
                    anchors.bottomMargin:20 
                    Repeater {
                        model: camNode.cameraList

                        delegate: CameraView {
                            cameraName: modelData.name
                            topic: modelData.topic
                            providerId: modelData.providerId
                        }
                    }
                }
            }

            // Info Panel (Right docked)
            Rectangle {
                width: parent.width * 0.2
                height: parent.height
                color: "#081e29"
                border.color: "#134357"
                radius: 6
                anchors.right: parent.right
                anchors.rightMargin: 10
                anchors.bottom: parent.bottom
                anchors.bottomMargin: 10
                
                GridLayout {
                    anchors.top: parent.top
                    anchors.left: parent.left
                    anchors.right: parent.right
                    anchors.margins: 10
                    columns: 1

                    Text {
                        text: "PRODUCTION INFORMATION"
                        color: "#5cf4f1"
                        font.bold: true
                        font.pixelSize: 20
                        padding: 10
                        Layout.alignment: Qt.AlignHCenter
                    }

                    Rectangle {
                        //Layout.topMargin: 10
                        height: 1
                        Layout.fillWidth: true
                        color: "#134357"
                    }

                    GridLayout{
                        //anchors.top: parent.top
                        Layout.topMargin: 20
                        columns: 2
                        rowSpacing: 20
                        columnSpacing: 10
                        Text {
                            text: "Product Name: "
                            color: "#5cf4f1"
                            font.pixelSize: 16
                        }
                        Text {
                            text: "FUNAI CARTRIDGE"
                            color: "#5cf4f1"
                            font.pixelSize: 16
                        }
                        Text {
                            text: "Production Quantity:"
                            color: "#5cf4f1"
                            font.pixelSize: 16
                        }
                        Text {
                            text: "270"
                            color: "#5cf4f1"
                            font.pixelSize: 16
                        }
                    }

                    // Item {
                    //     Layout.fillWidth: true
                    //     Layout.fillHeight: true

                    //     ChartView {
                    //         anchors.fill: parent
                    //         antialiasing: false
                    //         title: "Test Chart"
                    //         PieSeries {
                    //             PieSlice { label: "One"; value: 10 }
                    //             PieSlice { label: "Two"; value: 90 }
                    //         }
                    //     }
                    // }
                }
            }
        }

        Item {
            height: 45
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
