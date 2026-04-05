import discord

class Client(discord.Client):
    async def on_ready(self):
        print(f'Logged on as {self.user}!');
        
intents = discord.Intents.default();
intents.message_content = True;

#will fill in after to avoid secret being in repo
botToken = '';


#run bot
client = Client(intents = intents);
client.run(botToken);
