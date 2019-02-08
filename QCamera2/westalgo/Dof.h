#ifndef __SIL_DOF_H__
#define __SIL_DOF_H__

#include "Image.h"

using namespace wa;

namespace wa {

class Dof
{
public:
    Dof();
    ~Dof();
    Dof(const Dof& df);
    Dof& operator = (const Dof& df);

    /** @brief Set calibration data
     *
     * @param data calibration data
     * @param size size of data
     * @return 1 if success, otherwise falure
     */
    int setCalibData(const char* data, int size);

    /** @brief Changes buffer size.
     *
     * @param mainW reset width of pre-allocate buffer
     *        mainH reset height of pre-allocate buffer
     *
     * @raturn 1 if success, otherwise failure
     */
    int resize(int mainW, int mainH);

    /** @brief Set main image and aux image to fusion.
     *
     * @param mainImg main image
     *        auxImg  aux image
     */
    void setImages(Image& mainImg, Image& auxImg);

    /** @brief Run supernight algorithm.
     *
     * @param outImg output Image
     * @return 1 if success, otherwise failure
     */
    int run(int focusX, int focusY, float fNum, Image& outRefocused);

    /** @brief get width of depth
     *
     * @return width
     */
    int getWidthOfDepth();

    /** @brief get height of depth
     *
     * @return height
     */
    int getHeightOfDepth();

    /** @brief byte of per pixel
     *
     * 8U:  1
     * 16U: 2
     * 32F: 4
     *
     * @return byte of per pixel
     */
    unsigned int getElemSizeOfDepth();

    /** @brief depth format for google
     *
     * @return 0 RangeInverse
     *         1 RangeLinear
     */
    int getFormatOfDepth();

    /** @brief max deep value(cm) of depth image
     *
     * @return far value
     */
    float getFarOfDepth();

    /** @brief min deep value(cm) of depth image
     *
     * @return near value
     */
    float getNearOfDepth();

    void getDepth(Image& depthImg);

    /** @brief Get version number.
     *
     * @return version number
     */
    const char* getVersion() const;

private:
    class Impl;
    Impl *p;
}; // class Dof
} // namespace wa

#endif // __SIL_DOF_H__
