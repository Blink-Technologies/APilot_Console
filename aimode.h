#ifndef AIMODE_H
#define AIMODE_H

#include <QObject>

class AIMode : public QObject
{
    Q_OBJECT
public:
    explicit AIMode(QObject *parent = nullptr);
    static void UpdateParams(float *);
private:
    static void CalculateTargetToCenter(float *error_x, float *error_y);
    static void CalculateFCXY();
    static float cx, cy, fcx, fcy;

};

#endif // AIMODE_H
