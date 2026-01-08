import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Window 2.15

Window {
    id: settingsWindow
    width: 720
    height: 480
    title: "Settings"
    flags: Qt.Window
    color: "#0d1117"

    property var availableTopics: []
    property var selectedTopics: []

    Shortcut {
        sequence: "Escape"
        onActivated: settingsWindow.close()
    }

    Component.onCompleted: {
        Qt.callLater(() => {
            x = (mainWindow.width - width) / 2 + mainWindow.x
            y = (mainWindow.height - height) / 2 + mainWindow.y
        });

        availableTopics = camNode.getAvailableImageTopics();
        selectedTopics = camNode.cameraList.map(cam => cam.topic);
    }

    ColumnLayout {
        anchors.fill: parent
        spacing: 10

        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.leftMargin: 30
            Layout.rightMargin: 30
            Layout.topMargin: 20
            color: "transparent"
            border.color: "#134357"
            border.width: 1
            radius: 6

            GridLayout {
                columns: 1
                anchors.fill: parent
                anchors.margins: 10
                columnSpacing: 15

                Repeater {
                    model: camNode.cameraList

                    delegate: ColumnLayout {
                        property int index: model.index
                        spacing: 4

                        Text {
                            text: modelData.name
                            color: "#5cf4f1"
                            font.pixelSize: 16
                        }

                        ComboBox {
                            id: topicCombo
                            Layout.fillWidth: true
                            Layout.preferredHeight: 40

                            model: availableTopics
                            currentIndex: availableTopics.indexOf(modelData.topic)

                            onActivated: (topicIndex) => {
                                let selected = availableTopics[topicIndex]
                                selectedTopics[index] = selected
                                modelData.topic = selected
                            }

                            background: Rectangle {
                                color: "transparent"
                                border.color: "#134357"
                                border.width: 1
                                radius: 6
                            }

                            contentItem: Text {
                                text: topicCombo.displayText
                                font.pixelSize: 14
                                color: "#5cf4f1"
                                verticalAlignment: Text.AlignVCenter
                                horizontalAlignment: Text.AlignLeft
                                elide: Text.ElideRight
                                anchors.verticalCenter: parent.verticalCenter
                                anchors.left: parent.left
                                anchors.leftMargin: 10
                            }
                        }
                    }
                }
            }
        }

        Button {
            text: "Save"
            Layout.alignment: Qt.AlignRight
            Layout.topMargin: 0
            Layout.bottomMargin: 30
            Layout.rightMargin: 30
            Layout.preferredHeight: 40
            Layout.preferredWidth: 120

            contentItem: Text {
                text: "Save"
                color: "#5cf4f1"
                font.pixelSize: 14
                verticalAlignment: Text.AlignVCenter
                horizontalAlignment: Text.AlignHCenter
                anchors.fill: parent
            }

            background: Rectangle {
                radius: 6
                color: "transparent"
                border.color: "#134357"
                border.width: 2
            }

            onClicked: {
                for (let i = 0; i < selectedTopics.length; ++i) {
                    camNode.updateCameraTopic(i, selectedTopics[i])
                    console.log("Updated camera", i, "to topic:", selectedTopics[i])
                }
            }
        }
    }
}
