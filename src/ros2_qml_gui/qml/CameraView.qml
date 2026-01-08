import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Item {
    id: root
    Layout.fillWidth: true
    Layout.fillHeight: true

    property string cameraName: "Camera"
    property string topic: "/camera/image_raw"
    property string providerId: "cam"
    property int camIndex: 0

    Rectangle {
        anchors.fill: parent
        color: "#1e1e1e"
        border.color: "#134357"
        border.width: 1
        radius: 6

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 10
            spacing: 6

            Text {
                text: cameraName + " (" + topic + ")"
                color: "#6cf"
                padding: 4
                font.pixelSize: 14
                Layout.fillWidth: true
                wrapMode: Text.WordWrap
                horizontalAlignment: Text.AlignHCenter
            }

            Rectangle {
                id: previewBox
                Layout.fillWidth: true
                Layout.fillHeight: true
                color: "#2a2a2a"
                // border.color: "#444"
                radius: 4

                Item {
                    id: aspectContainer
                    width: previewBox.width
                    height: previewBox.width * 3 / 4
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.verticalCenter: parent.verticalCenter
                    clip: true

                    Image {
                        id: camImage
                        anchors.fill: parent
                        fillMode: Image.PreserveAspectFit
                        smooth: true
                        source: "image://" + providerId + "/" + camIndex

                        Timer {
                            interval: 1000 / 30
                            running: true
                            repeat: true
                            onTriggered: camImage.source = "image://" + providerId + "/" + camIndex + "?" + Date.now()
                        }
                    }
                }
            }
        }
    }
}
