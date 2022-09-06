import torch
import torch.nn as nn

class PoseModel(nn.Module):
    def __init__(self):
        super(PoseModel, self).__init__()
        self.conv_layer = nn.Sequential(
            # 128 128
            nn.Conv2d(1, 16, 3, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(16, 16, 3, padding=1),
            nn.ReLU(inplace=True),
            nn.AvgPool2d(2),
            # 64 64
            nn.Conv2d(16, 32, 3, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(32, 32, 3, padding=1),
            nn.ReLU(inplace=True),
            nn.AvgPool2d(2),
            # 32 32
            nn.Conv2d(32, 64, 3, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(64, 64, 3, padding=1),
            nn.ReLU(inplace=True),
            nn.AvgPool2d(2),
            # 16 16
            nn.Conv2d(64, 128, 3, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(128, 128, 3, padding=1),
            nn.ReLU(inplace=True),
            nn.AvgPool2d(2),
            # 8 8
            nn.Conv2d(128, 256, 3, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(256, 256, 3, padding=1),
            nn.ReLU(inplace=True),
            nn.Flatten(),
        )
        self.linear_layers = nn.Sequential(
            nn.Linear(in_features=256 * 8 * 8, out_features=4000),
            nn.ReLU(),
            nn.Linear(in_features=4000, out_features=1000),
            nn.ReLU(),
            nn.Linear(in_features=1000, out_features=100),
            nn.ReLU(),
            nn.Linear(in_features=100, out_features=2),
        )

    def forward(self, input):
        output = self.conv_layer(input)
        output = self.linear_layers(output)
        return output
