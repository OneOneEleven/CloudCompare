/**
 *
 *
 *
 *
 *
 *
 * 
 *
 */

//CC
#include <ccOverlayDialog.h>

//Local
#include <ui_LabelingUI.h>

class ccLabelingUI :public ccOverlayDialog
{
	Q_OBJECT
public:
	explicit ccLabelingUI(QWidget* parent);

signals:
	void activatePick();

private:
	Ui::LabelingUI* _ui;
};
