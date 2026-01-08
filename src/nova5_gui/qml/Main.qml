import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Controls.Material 2.15

ApplicationWindow {
  id: win
  visible: true
  width: 1000
  height: 640
  title: "Nova5 GUI"

  Material.theme: Material.Dark
  Material.primary: Material.Teal

  ColumnLayout {
    anchors.fill: parent
    anchors.margins: 12
    spacing: 12

    Label {
      text: nova5.busy ? "BUSY..." : "Ready"
      color: nova5.busy ? "orange" : "lightgreen"
      font.bold: true
    }

    Loader { source: "qrc:/qml/Nova5Panel.qml" }
    // Bạn có thể add thêm các panel khác ở đây
  }
}
