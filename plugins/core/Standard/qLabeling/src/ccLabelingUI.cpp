#include <ccLabelingUI.h>

ccLabelingUI::ccLabelingUI(QWidget* parent)
	: ccOverlayDialog(parent)
	, _ui(new Ui::LabelingUI)
{
	_ui->setupUi(this);


	connect(_ui->btnPickPoint, &QPushButton::clicked, this, &ccLabelingUI::activatePick);

}
