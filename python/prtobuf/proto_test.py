#! /usr/bin/env python
import addressbook_pb2
from google.protobuf.text_format import MessageToString
from google.protobuf.text_format import Parse

address_book = addressbook_pb2.AddressBook()
# person = address_book.people.add()
#
# person.id = 1
# person.name = "safly"
# person.email = "safly@qq.com"
# person.money = 1000.11
# person.work_status = True
#
# phone_number = person.phones.add()
# phone_number.number = "123456"
# phone_number.type = addressbook_pb2.MOBILE
#
# maps = person.maps
# maps.mapfield[1] = 1
# maps.mapfield[2] = 2
#
# # 序列化
# serializeToString = address_book.SerializeToString()
# print(serializeToString, type(serializeToString))
#
# address_book.ParseFromString(serializeToString)
# print(address_book)
#
# for person in address_book.people:
#   print("p_id{},p_name{},p_email{},p_money{},p_workstatu{}"
#         .format(person.id, person.name, person.email, person.money, person.work_status))
#
#   for phone_number in person.phones:
#     print(phone_number.number, phone_number.type)
#
#   for key in person.maps.mapfield:
#     print(key, person.maps.mapfield[key])

with open("output.txt", 'r') as f:
  Parse(f.read(), address_book)
# print(address_book)
print("\n")
address_book.people[0].id = 3
with open("output_2.txt", 'w') as f:
  f.write(MessageToString(address_book))

# print(address_book)
# if vehicle_proto.gnss.installation.data_port.HasField('udp'):
print(len(address_book.people))

