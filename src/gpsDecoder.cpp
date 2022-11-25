//	int ch1,ch2,ch3, counter, ch;
//	int lla[7];
//	bool checkUbloxPattern = false;
//	bool readUblox = false;
//	while (true)
//	{
//		ch1 = huart1->read();
//		ch2 = huart1->read();
//		ch3 = huart1->read();
//		if (ch1 == '$' && ch2 == 'G' && ch3 == 'P')
//		{
//			int first = huart1->read();
//			int second = huart1->read();
//			int third = huart1->read();
//			if (first == 'G' && second == 'L' && third == 'L')
//			{
//				// LLA msg next 8 bytes seperated by comma, then \r
//				for (int i=0; i<8; i++)
//				{
//					huart1->read(); // read comma and ignore
//					ch = huart1->read();
//					lla[i] = ch;
//				}
//			}
//		}
//		
//		
//	}
