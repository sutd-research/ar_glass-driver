import rospy
import os
import string,cgi,time
from os import curdir, sep
from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
from urlparse import urlparse, parse_qs


class RequestHandler(BaseHTTPRequestHandler):

    def do_GET(self):
        """
            Send Image to AR Glass
        """
        rospy.loginfo("Somebody made a POST request.")
        
        try:
            nameToSend = urlparse(self.path).query
            print(nameToSend)
            self.send_response(200)
            self.send_header('Content-type','image/png')
            self.end_headers()
            self.wfile.write(image)
            rospy.loginfo("File sent.  *******************************02")
            return
            
        except IOError:
            self.send_error(404,'File Not Found: ')
     
    def do_POST(self):
        """
            Capture Image from AR Glass
        """
        global rootnode
        try:
            ctype, pdict = cgi.parse_header(self.headers.getheader('content-type'))
            print (ctype)
            print (pdict)
            if ctype == 'multipart/form-data':
                query=cgi.parse_multipart(self.rfile, pdict)
            elif ctype == 'application/x-www-form-urlencoded':
                length = int(self.headers.getheader('content-length'))
                query = cgi.parse_qs(self.rfile.read(length), keep_blank_values=1)

            self.send_response(301)
            self.send_header('Content-type','image/png')
            self.end_headers()

            nameReceived = query.get('name')
            upfilecontent = query.get('image')
            print(nameReceived[0])
            image = upfilecontent[0].decode('base64')
            rospy.loginfo("File received.  *******************************01")
            return image
            
        except Exception:
            rospy.logerr("Exception")

def main():
    global sendCount
    sendCount = 1
    try:
        server = HTTPServer(('192.168.0.177', 80), RequestHandler)
        print ('started httpserver...')
        server.serve_forever()
        
    except KeyboardInterrupt:
        print ('^C received, shutting down server')
        server.socket.close()

if __name__ == '__main__':
    main()

