import cv2 as cv
import fitz
row = 10
col = 8
marker_num = int(row / 2 * col)
picture_num = 20
width = 13000
high = 10000
boundry = 500
tx = 100
ty = 100
tsize = 5
checker_size = 90
marker_size = 60
font = cv.FONT_HERSHEY_SCRIPT_SIMPLEX

def img2pdf(img_in,img_out):
  doc = fitz.open()

  imgdoc = fitz.open(img_in)
  pdfbytes = imgdoc.convertToPDF()
  imgpdf = fitz.open('pdf', pdfbytes)
  doc.insertPDF(imgpdf)
  doc.save(img_out)
  doc.close



for i in range(picture_num):
  dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_1000)
  input_ids = [i for i in range(i * marker_num, i * marker_num + marker_num)]
  board = cv.aruco.CharucoBoard_create(row, col, 0.15, 0.1, dictionary)
  board.ids = input_ids
  img = cv.aruco_CharucoBoard.draw(board, (width, high), marginSize=boundry, borderBits=1)
  text = "row x col : {}x{} | checker size : {} mm | marker size : {} mm | dictionary : DICT_4X4_1000 " \
         "{:^3d}--{:^3d}  |  -- |  {:^3d}--{:^3d}".format(row, col, checker_size, marker_size, i * marker_num,
                                                              i * marker_num + int(row / 2) - 1,
                                                              i * marker_num + marker_num - 1 - int(row / 2),
                                                              i * marker_num + marker_num - 1)
  cv.putText(img, text, (tx, ty), cv.FONT_HERSHEY_PLAIN, tsize, (0, 0, 0), tsize)

  cv.imwrite('charuco_{}.png'.format(i), img)
  img2pdf('charuco_{}.png'.format(i), 'charuco_{}.pdf'.format(i))
  # cv.imshow("img", img)
  # cv.waitKey()

  # img_2 = cv.cvtColor(img, cv.COLOR_GRAY2RGB)
  # img_3 = cv.cvtColor(img, cv.COLOR_GRAY2RGB)
  # corners, ids, rejectedImgPoints = cv.aruco.detectMarkers(img, dictionary)
  # cv.aruco.drawDetectedMarkers(img_2, corners, ids, borderColor=None)
  # cv.imshow("img_2", img_2)
  # cv.waitKey()

  # cv.imshow("img_3", img_3)
  # cv.waitKey(0)

  # print(corners)
  # print(ids)
  # print(rejectedImgPoints)
  # if (len(ids) > 0):
  #   cv.aruco.estimatePoseSingleMarkers()
